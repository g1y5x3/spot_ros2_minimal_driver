import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse

from rclpy.action import ActionClient
from spot_action.action import MoveRelativeXY 

class PathPlanningClient(Node):
    def __init__(self):
        super().__init__('path_planning_client')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._move_client = ActionClient(self, MoveRelativeXY, 'move_relative_xy')

    def follow_path(self, msgs: list):
        for msg in msgs:
            goal_msg = MoveRelativeXY.Goal()
            goal_msg.x = msg.x
            goal_msg.y = msg.y
            goal_msg.yaw = msg.yaw

            self.get_logger().info(
            f"Transformed goal:\n"
            f"x: {goal_msg.x:.2f}, "
            f"y: {goal_msg.y:.2f}"
            f"yaw: {goal_msg.yaw:.2f}"
        )

        self._move_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

