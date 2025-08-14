#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import sensor_msgs_py.point_cloud2 as pc2

from moveit_msgs.srv import GetPositionIK, GetCartesianPath, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import (
    PositionIKRequest,
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotState,
    MoveItErrorCodes,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SphereMotionPlanner(Node):
    def __init__(self):
        super().__init__('sphere_motion_planner')

        # Home joint state
        self.home = JointState(
            name=[
                'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint',    'wrist_2_joint',
                'wrist_3_joint',    'shoulder_pan_joint',
            ],
            position=[
                -1.6199397577352357,
                 1.3999678228092844,
                -1.199945876570465,
                -1.6000818166478072,
                -0.10990594776300713,
                 1.5400809756241274,
            ],
        )

        self.declare_parameter('approach_weight', 0.5)
        self.declare_parameter('final_weight',    0.5)
        self.declare_parameter('cone_angle',      45.0)     # degrees
        self.declare_parameter('group_name',      'ur_manipulator')
        self.declare_parameter('cone_points_topic', 'sampled_cone_points')  # topic name

        self.approach_weight = self.get_parameter('approach_weight').get_parameter_value().double_value
        self.final_weight    = self.get_parameter('final_weight').get_parameter_value().double_value
        self.cone_angle      = self.get_parameter('cone_angle').get_parameter_value().double_value
        self.group_name      = self.get_parameter('group_name').get_parameter_value().string_value
        self.sample_topic    = self.get_parameter('cone_points_topic').get_parameter_value().string_value

        # cone‑sampling parameter (half‑angle in degrees)
        self.cos_cone   = math.cos(math.radians(self.cone_angle))

        # publisher for sampled cone points
        self.sample_pub = self.create_publisher(PointCloud2, self.sample_topic, 1)

        # MoveIt! service clients
        self.ik_cli     = self.create_client(GetPositionIK,    'compute_ik')
        self.plan_cli   = self.create_client(GetMotionPlan,     'plan_kinematic_path')
        self.cart_cli   = self.create_client(GetCartesianPath,  'compute_cartesian_path')
        self.fk_cli     = self.create_client(GetPositionFK,     'compute_fk')

        for cli, name in [
            (self.ik_cli,   'compute_ik'),
            (self.plan_cli, 'plan_kinematic_path'),
            (self.cart_cli, 'compute_cartesian_path'),
            (self.fk_cli,   'compute_fk')
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service '{name}' not available")
                rclpy.shutdown()
                return

    def compute_fk(self, joint_state: JointState) -> PoseStamped:
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names   = ['gripper_link']
        req.robot_state     = RobotState(joint_state=joint_state)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().pose_stamped[0]

    def compute_manipulability(self, joint_state: JointState) -> float:
        delta = 1e-4
        ps0   = self.compute_fk(joint_state)
        p0    = np.array([ps0.pose.position.x,
                          ps0.pose.position.y,
                          ps0.pose.position.z])
        n     = len(joint_state.position)
        J     = np.zeros((3, n))
        for i in range(n):
            js = JointState(name=joint_state.name,
                            position=list(joint_state.position))
            js.position[i] += delta
            psi = self.compute_fk(js)
            pi  = np.array([psi.pose.position.x,
                            psi.pose.position.y,
                            psi.pose.position.z])
            J[:, i] = (pi - p0) / delta
        m2 = np.linalg.det(J @ J.T)
        return float(math.sqrt(max(m2, 0.0)))

    def quaternion_between_vectors(self, ref, tgt):
        axis = np.cross(ref, tgt)
        n    = np.linalg.norm(axis)
        if n < 1e-8:
            return (0, 0, 0, 1) if np.dot(ref, tgt) > 0 else (1, 0, 0, 0)
        axis /= n
        angle = math.acos(np.clip(np.dot(ref, tgt), -1, 1))
        s     = math.sin(angle / 2)
        return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))

    def plan_for_center(self,
                        center: Point,
                        num_pts: int = 20,
                        radius: float = 0.2):
        """
        Sample cone points, publish, then do:
        1) IK + joint-space plan from home->approach
        2) Cartesian path from actual approach endpoint->center,
           shifting every center‐segment timestamp by that endpoint time
        3) Return both trajectories and score
        """
        # compute cone axis
        home_ps = self.compute_fk(self.home)
        H = home_ps.pose.position
        axis = np.array([H.x - center.x,
                         H.y - center.y,
                         H.z - center.z])
        axis /= np.linalg.norm(axis)

        # sample cone points & publish
        pts = self._generate_cone_points(center, axis, radius, num_pts)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        cloud = pc2.create_cloud_xyz32(header,
                                        [[p.x, p.y, p.z] for p in pts])
        self.sample_pub.publish(cloud)

        best_score    = -1.0
        best_approach = None
        best_center   = None

        for sample in pts:
            # build approach & center poses
            dir_vec  = np.array([center.x - sample.x,
                                 center.y - sample.y,
                                 center.z - sample.z])
            dir_norm = dir_vec / np.linalg.norm(dir_vec)
            q = self.quaternion_between_vectors(ref=np.array([0,0,1]),
                                                tgt=dir_norm)
            app = PoseStamped()
            app.header.frame_id  = 'base_link'
            app.pose.position    = sample
            app.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            cen = PoseStamped()
            cen.header.frame_id  = 'base_link'
            cen.pose.position    = center
            cen.pose.orientation = app.pose.orientation

            # 1) IK: home -> approach
            ik1 = PositionIKRequest(
                group_name   = self.group_name,
                ik_link_name = 'gripper_link',
                robot_state  = RobotState(joint_state=self.home),
                pose_stamped = app,
                timeout      = Duration(sec=1),
            )
            fut1 = self.ik_cli.call_async(GetPositionIK.Request(ik_request=ik1))
            rclpy.spin_until_future_complete(self, fut1)
            res1 = fut1.result()
            if res1.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            js_app = res1.solution.joint_state

            # 2) joint-space plan: home -> approach
            pv = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
            bv = BoundingVolume(primitives=[pv],
                                primitive_poses=[app.pose])
            pos_c = PositionConstraint(header=app.header,
                                       link_name='gripper_link',
                                       constraint_region=bv,
                                       weight=1.0)
            ori_c = OrientationConstraint(header=app.header,
                                          link_name='gripper_link',
                                          orientation=app.pose.orientation,
                                          absolute_x_axis_tolerance=0.05,
                                          absolute_y_axis_tolerance=0.05,
                                          absolute_z_axis_tolerance=0.05,
                                          weight=1.0)
            cons = Constraints(position_constraints=[pos_c],
                               orientation_constraints=[ori_c])

            mp_req = MotionPlanRequest(
                group_name                      =self.group_name,
                start_state                     =RobotState(joint_state=self.home),
                goal_constraints                =[cons],
                num_planning_attempts           =10,
                allowed_planning_time           =5.0,
                max_velocity_scaling_factor     =0.10,
                max_acceleration_scaling_factor =0.10,
            )
            fut_mp = self.plan_cli.call_async(
                GetMotionPlan.Request(motion_plan_request=mp_req))
            rclpy.spin_until_future_complete(self, fut_mp)
            mp_res = fut_mp.result()
            if mp_res.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            approach_traj = mp_res.motion_plan_response.trajectory.joint_trajectory

            # record endpoint of approach
            end_pt = approach_traj.points[-1]
            end_positions = list(end_pt.positions)
            end_time      = end_pt.time_from_start

            # 3) IK: approach -> center (to seed Cartesian path)
            ik2 = PositionIKRequest(
                group_name   = self.group_name,
                ik_link_name = 'gripper_link',
                robot_state  = RobotState(joint_state=js_app),
                pose_stamped = cen,
                timeout      = Duration(sec=1),
            )
            fut2 = self.ik_cli.call_async(GetPositionIK.Request(ik_request=ik2))
            rclpy.spin_until_future_complete(self, fut2)
            res2 = fut2.result()
            if res2.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            js_cent = res2.solution.joint_state

            # compute manipulability score
            m_app  = self.compute_manipulability(js_app)
            m_cent = self.compute_manipulability(js_cent)
            score  = self.approach_weight * m_app + self.final_weight * m_cent

            # 4) Cartesian path: actual endpoint -> center
            cart_req = GetCartesianPath.Request()
            cart_start = JointState()
            cart_start.name     = approach_traj.joint_names
            cart_start.position = end_positions
            cart_req.start_state      = RobotState(joint_state=cart_start)
            cart_req.group_name       = self.group_name
            cart_req.waypoints        = [app.pose, cen.pose]
            cart_req.max_step         = 0.01
            cart_req.jump_threshold   = 0.0
            cart_req.avoid_collisions = True
            futc = self.cart_cli.call_async(cart_req)
            rclpy.spin_until_future_complete(self, futc)
            cres = futc.result()
            if cres.fraction < 1.0:
                continue
            center_traj = cres.solution.joint_trajectory

            # **simple timestamp shift**: add the approach end_time to every center point
            for pt in center_traj.points:
                pt.time_from_start.sec     += end_time.sec
                pt.time_from_start.nanosec += end_time.nanosec
                if pt.time_from_start.nanosec >= 1_000_000_000:
                    pt.time_from_start.sec     += 1
                    pt.time_from_start.nanosec -= 1_000_000_000

            # keep the best‐scoring pair
            if score > best_score:
                best_score    = score
                best_approach = approach_traj
                best_center   = center_traj

        return best_approach, best_center, best_score

    def _generate_cone_points(self,
                              center: Point,
                              axis: np.ndarray,
                              radius: float,
                              num_points: int):
        """
        Uniform cap sampling of a cone
        """
        pts = []
        qx, qy, qz, qw = self.quaternion_between_vectors(
            ref=np.array([0,0,1]), tgt=axis)
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx*qx+qy*qy)]
        ])
        inc = math.pi * (3.0 - math.sqrt(5.0))
        cos_min = self.cos_cone
        for k in range(num_points):
            z = 1.0 - (k/(num_points-1)) * (1.0 - cos_min)
            r = math.sqrt(max(0.0, 1.0 - z*z))
            phi = k * inc
            x_loc = r * math.cos(phi)
            y_loc = r * math.sin(phi)
            v_world = R.dot([x_loc, y_loc, z])
            pts.append(Point(
                x=center.x + v_world[0] * radius,
                y=center.y + v_world[1] * radius,
                z=center.z + v_world[2] * radius
            ))
        return pts


def main(args=None):
    rclpy.init(args=args)
    node = SphereMotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
