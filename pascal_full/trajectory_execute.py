import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory

class PlanExecutor(Node):
    """
    Subscribes to /selected_trajectory (JointTrajectory), then
    sends an ExecuteTrajectory goal to MoveIt for execution.
    """
    def __init__(self):
        super().__init__('plan_executor')

        self.declare_parameter('trajectory_topic', '/pascal_cached_trajectory')
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value

        # ROS 2 ExecuteTrajectory action client
        self._client = ActionClient(self,
                                    ExecuteTrajectory,
                                    '/execute_trajectory')
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory server not available")
            rclpy.shutdown()
            return

        # subscribe to selected trajectory
        self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.on_trajectory,
            10
        )

    def on_trajectory(self, traj: JointTrajectory):
        self.get_logger().info(
            f"Received trajectory with {len(traj.points)} points, sending to action..."
        )

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj

        send = self._client.send_goal_async(goal)
        send.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Goal accepted, waiting result...")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result().result
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Execution succeeded!")
        else:
            self.get_logger().error(
                f"Execution failed: {res.error_code.val}"
            )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PlanExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
