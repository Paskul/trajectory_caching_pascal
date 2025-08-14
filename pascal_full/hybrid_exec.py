#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 1) HybridPlanner action
from moveit_msgs.action import HybridPlanner

# 2) Messages for the motion sequence
from moveit_msgs.msg import (
    MotionSequenceRequest,
    MotionSequenceItem,
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
    PlanningScene,
    CollisionObject
)
from moveit_msgs.srv import ApplyPlanningScene

# 3) Primitives and poses for our obstacle
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class HybridPlanClient(Node):
    def __init__(self):
        super().__init__('hybrid_plan_client')

        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('pipeline_id', 'ompl')

        # Action client for hybrid planning
        self._client = ActionClient(
            self,
            HybridPlanner,
            '/hybrid_planning/run_hybrid_planning'
        )
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("HybridPlanner action server not available")
            rclpy.shutdown()
            return

        # Service client to push PlanningScene diffs
        self._scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        if not self._scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("apply_planning_scene service not available")
            rclpy.shutdown()
            return

        # Kick off our test
        self.send_test_goal()

    def send_test_goal(self):
        # --- 1) Build the MotionPlanRequest ---
        mp_req = MotionPlanRequest()

        # workspace must be valid
        mp_req.workspace_parameters = WorkspaceParameters(
            header=mp_req.workspace_parameters.header,
            min_corner=mp_req.workspace_parameters.min_corner.__class__(x=-1.0, y=-1.0, z=-1.0),
            max_corner=mp_req.workspace_parameters.max_corner.__class__(x= 1.0, y= 1.0, z= 1.0),
        )
        mp_req.workspace_parameters.header.frame_id = 'base_link'


        # choose your group and pipeline explicitly
        mp_req.group_name = self.get_parameter('group_name').get_parameter_value().string_value     
        #mp_req.pipeline_id                 = 'move_group'
        mp_req.pipeline_id = self.get_parameter('pipeline_id').get_parameter_value().string_value
        
        #mp_req.planner_id                  = 'RRTstarkConfigDefault'
        mp_req.planner_id                  = 'geometric::RRTstar'
        
        mp_req.num_planning_attempts       = 4
        mp_req.allowed_planning_time       = 5.0
        mp_req.max_velocity_scaling_factor    = 0.0002
        mp_req.max_acceleration_scaling_factor = 0.0002

        # target pose (your previously chosen test/home pose)
        home = {
            'shoulder_pan_joint':   1.571,
            'shoulder_lift_joint': -1.575,
            'elbow_joint':          1.62,
            'wrist_1_joint':       3.08,
            'wrist_2_joint':       -1.57,
            'wrist_3_joint':       0.0,
        }
        goal = Constraints()
        goal.joint_constraints = []
        for name, pos in home.items():
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            goal.joint_constraints.append(jc)
        mp_req.goal_constraints = [goal]

        # --- 2) Wrap in a MotionSequenceRequest ---
        seq_item = MotionSequenceItem()
        seq_item.req          = mp_req
        seq_item.blend_radius = 0.0

        seq_req = MotionSequenceRequest()
        seq_req.items = [seq_item]

        # --- 3) Send the HybridPlanner goal ---
        goal_msg = HybridPlanner.Goal()
        goal_msg.planning_group  = self.get_parameter('group_name').get_parameter_value().string_value
        goal_msg.motion_sequence = seq_req

        send_goal = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback
        )
        send_goal.add_done_callback(self.on_goal_response)
        self.get_logger().info("Sent hybrid‐planning goal → obstacle insertion in 1.5 s")

        # schedule our obstacle insertion
        #self._obstacle_timer = self.create_timer(0.5, self._insert_obstacle)

    def _insert_obstacle(self):
        # cancel further calls
        self._obstacle_timer.cancel()

        # define a blocking box
        co = CollisionObject()
        co.id            = 'blocking_box'
        co.header.frame_id = 'base_link'
        box = SolidPrimitive(type=SolidPrimitive.BOX,
                             dimensions=[0.1, 0.1, 0.1])
        co.primitives       = [box]
        p = Pose()
        p.orientation.w = 1.0
        p.position.x    = 0.5
        p.position.y    = 0.0
        p.position.z    = 0.5
        co.primitive_poses = [p]
        co.operation      = CollisionObject.ADD

        # wrap in a PlanningScene diff
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        req = ApplyPlanningScene.Request(scene=scene)
        self._scene_client.call_async(req)

        self.get_logger().warn("🚧 Inserted blocking obstacle into planning scene")

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[HybridPlanner feedback] {fb}")

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("HybridPlanner goal rejected")
            return
        self.get_logger().info("HybridPlanner goal accepted, waiting for result…")
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("✅ Hybrid planning succeeded!")
        else:
            self.get_logger().error(f"❌ Hybrid planning failed: {result.error_message}")
        # clean shutdown
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HybridPlanClient()
    rclpy.spin(node)
    # shutdown happens in on_result()
    

if __name__ == '__main__':
    main()
