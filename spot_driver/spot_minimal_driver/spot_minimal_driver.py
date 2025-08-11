# Copyright 2025 Yixiang Gao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A minimal ROS 2 driver for Boston Dynamics Spot robot."""

import math
import time
from typing import Optional

import bosdyn.client
import bosdyn.client.util
import rclpy
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.api.robot_state_pb2 import RobotState
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (
    GRAV_ALIGNED_BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    get_a_tform_b,
    get_se2_a_tform_b,
)
from bosdyn.client.lease import Error as LeaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.math_helpers import SE2Pose, SE3Pose, SE3Velocity
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

#
from bosdyn.client.world_object import WorldObjectClient, world_object_pb2
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from spot_action.action import MoveRelativeXY
from spot_srvs.srv import GetTransform

#


class SpotROS2Driver(Node):
    """A minimal ROS 2 driver for Boston Dynamics Spot robot."""

    def __init__(self):
        """Initialize the Spot ROS 2 driver node."""
        super().__init__("spot_driver_node")

        self.declare_parameter("hostname", "192.168.80.3")
        self.hostname = self.get_parameter("hostname").get_parameter_value().string_value
        # TODO: Add parameter for robot username and password if needed

        # load the user-defined odometry frame
        self.declare_parameter("odometry_frame", "odom")
        self.odom_frame = self.get_parameter("odometry_frame").get_parameter_value().string_value
        if self.odom_frame not in [ODOM_FRAME_NAME, VISION_FRAME_NAME]:
            self.get_logger().error(f'Invalid odometry frame: {self.odom_frame}. Using default "odom".')
            self.odom_frame = ODOM_FRAME_NAME
        else:
            self.get_logger().info(f"Using odometry frame: {self.odom_frame}")

        self.robot: Optional[bosdyn.client.robot.Robot] = None
        self.lease_keep_alive: Optional[LeaseKeepAlive] = None
        self.estop_keep_alive: Optional[EstopKeepAlive] = None
        self.robot_state_client: Optional[RobotStateClient] = None
        self.command_client: Optional[RobotCommandClient] = None
        self.world_object_client: Optional[WorldObjectClient] = None

        try:
            # Robot initialization
            sdk = bosdyn.client.create_standard_sdk("SpotROS2DriverClient")
            self.robot = sdk.create_robot(self.hostname)
            # bosdyn.client.util.authenticate(self.robot)
            self.robot.authenticate("admin", "jr7oike8y86g")
            self.robot.time_sync.wait_for_sync()

            # NOTE: Not sure if this is necessary
            assert not self.robot.is_estopped(), (
                "Robot is estopped. Please use an external E-Stop client, "
                "such as the estop SDK example, to configure E-Stop."
            )

            self.get_logger().info("Successfully authenticated and connected to the robot.")

            # Create SDK clients
            self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
            self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
            estop_client = self.robot.ensure_client(EstopClient.default_service_name)
            self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)
            self.get_logger().info("Robot clients created.")

            # Lease management
            self.lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
            self.get_logger().info("Acquired lease.")

            # Acquire E-Stop
            estop_endpoint = EstopEndpoint(estop_client, "SpotROS2DriverEStop", 10.0)
            estop_endpoint.force_simple_setup()
            self.estop_keep_alive = EstopKeepAlive(estop_endpoint)
            self.get_logger().info("Acquired E-Stop.")

            time.sleep(3.0)

            # Power on and Stand Robot
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed."
            self.get_logger().info("Robot powered on.")

            blocking_stand(self.command_client, timeout_sec=10)
            self.get_logger().info("Robot standing.")

        except (RpcError, ResponseError, LeaseError) as e:
            self.get_logger().error(f"Failed to connect to the robot: {e}")
            raise

        # ROS 2 publishers and subscribers
        self.s_tf_broadcaster = StaticTransformBroadcaster(self)
        self.d_tf_broadcaster = TransformBroadcaster(self)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.robot_state_publisher = self.create_timer(
            0.1, self.publish_robot_state, callback_group=ReentrantCallbackGroup()
        )

        # Action server initialization
        self._action_server = ActionServer(
            self,
            MoveRelativeXY,
            "move_relative_xy",
            execute_callback=self.move_relative_xy,
            callback_group=ReentrantCallbackGroup(),
        )

        self.srv = self.create_service(GetTransform, "get_fiducial_transform", self.handle_get_transform)

    def handle_get_transform(self, request, response):
        fiducials = self.world_object_client.list_world_objects([world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects
        if not fiducials:
            self.get_logger().warn("No AprilTag fiducials found.")
            return

        tform_odom_fiducial = get_a_tform_b(fiducials[0].transforms_snapshot, self.odom_frame, "filtered_fiducial_200")

        tform_fiducial_odom = tform_odom_fiducial.inverse()

        t = TransformStamped()
        # TODO: sync with the robot's internal time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "filtered_fiducial_200"
        t.child_frame_id = f"odom_{self.odom_frame}"
        t.transform.translation.x = tform_fiducial_odom.position.x
        t.transform.translation.y = tform_fiducial_odom.position.y
        t.transform.translation.z = tform_fiducial_odom.position.z
        t.transform.rotation.x = tform_fiducial_odom.rotation.x
        t.transform.rotation.y = tform_fiducial_odom.rotation.y
        t.transform.rotation.z = tform_fiducial_odom.rotation.z
        t.transform.rotation.w = tform_fiducial_odom.rotation.w

        self.s_tf_broadcaster.sendTransform(t)
        response.transform = t
        return response

    def move_relative_xy(self, goal_handle: ServerGoalHandle):
        """Execute the move to relative [x, y, yaw] action."""
        goal = goal_handle.request
        self.get_logger().info(f"Executing goal: x={goal.x}, y={goal.y}, theta={goal.yaw}")

        distance = math.sqrt(goal.x**2 + goal.y**2)
        max_vel = 1.0  # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/spot/robot_command.proto#L66
        estimated_time = (distance / max_vel) + 5.0  # Add 5 second for safety margin

        try:
            transforms = self.robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

            # convert the goal pose from robot body frame to odom frame
            body_tform_goal = SE2Pose(x=goal.x, y=goal.y, angle=goal.yaw)
            odom_tform_body = get_se2_a_tform_b(transforms, self.odom_frame, GRAV_ALIGNED_BODY_FRAME_NAME)
            odom_tfrom_goal = odom_tform_body * body_tform_goal

            command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=odom_tfrom_goal.x,
                goal_y=odom_tfrom_goal.y,
                goal_heading=odom_tfrom_goal.angle,
                frame_name=self.odom_frame,
            )

            cmd_id = self.command_client.robot_command(command, end_time_secs=time.time() + estimated_time)

            # feedback_msg = MoveRelativeXY.Feedback()

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled.")
                    self.command_client.robot_command(RobotCommandBuilder.stop_command())
                    return MoveRelativeXY.Result(success=False)

                feedback = self.command_client.robot_command_feedback(cmd_id)
                mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

                if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    self.get_logger().error("Failed to reach the goal.")
                    goal_handle.abort()
                    return MoveRelativeXY.Result(success=False)

                # TODO: Add feedback publishing

                traj_feedback = mobility_feedback.se2_trajectory_feedback
                if (
                    traj_feedback.status == traj_feedback.STATUS_AT_GOAL
                    and traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED
                ):
                    self.get_logger().info("Arrived at the goal.")
                    goal_handle.succeed()
                    return MoveRelativeXY.Result(success=True)

                time.sleep(0.1)  # Check status at 10 Hz

        except (RpcError, ResponseError) as e:
            self.get_logger().error(f"Error during action execution: {e}")
            goal_handle.abort()
            return MoveRelativeXY.Result(success=False)

    def publish_robot_state(self):
        """Periodic publish robot data (if connected)."""
        robot_state: RobotState = self.robot_state_client.get_robot_state()
        odom_tfrom_body = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot, self.odom_frame, GRAV_ALIGNED_BODY_FRAME_NAME
        )
        self.publish_transform(odom_tfrom_body, f"odom_{self.odom_frame}", "base_link")

        odom_vel_of_body = robot_state.kinematic_state.velocity_of_body_in_odom
        self.publish_odometry(odom_tfrom_body, odom_vel_of_body, f"odom_{self.odom_frame}", "base_link")

        # TODO: Read internal robot inertial measurement and publish it but it's blocked by the Joint API license.

        # self.publish_transform(odom_tfrom_body, 'odom', 'base_link')

    def publish_odometry(self, odom_tfrom_body: SE3Pose, odom_vel_of_body: SE3Velocity, header: str, child: str):
        """Publish the odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = header
        odom_msg.child_frame_id = child

        odom_msg.pose.pose.position.x = odom_tfrom_body.position.x
        odom_msg.pose.pose.position.y = odom_tfrom_body.position.y
        odom_msg.pose.pose.position.z = odom_tfrom_body.position.z
        odom_msg.pose.pose.orientation.x = odom_tfrom_body.rotation.x
        odom_msg.pose.pose.orientation.y = odom_tfrom_body.rotation.y
        odom_msg.pose.pose.orientation.z = odom_tfrom_body.rotation.z
        odom_msg.pose.pose.orientation.w = odom_tfrom_body.rotation.w

        self.odom_publisher.publish(odom_msg)

    def publish_transform(self, tfrom: SE3Pose, header: str, child: str):  # type: ignore
        """Publish the transform from ODOM to BODY frame."""
        t = TransformStamped()
        # TODO: sync with the robot's internal time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = header
        t.child_frame_id = child
        t.transform.translation.x = tfrom.position.x
        t.transform.translation.y = tfrom.position.y
        t.transform.translation.z = tfrom.position.z
        t.transform.rotation.x = tfrom.rotation.x
        t.transform.rotation.y = tfrom.rotation.y
        t.transform.rotation.z = tfrom.rotation.z
        t.transform.rotation.w = tfrom.rotation.w

        self.d_tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        """Convert a Twist message to a robot velocity command and send it."""
        v_x, v_y, v_rot = msg.linear.x, msg.linear.y, msg.angular.z

        command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot)

        try:
            # Send the command to the robot
            self.command_client.robot_command(command, end_time_secs=time.time() + 0.5)
            self.get_logger().debug(f"Sent velocity command: v_x={v_x}, v_y={v_y}, v_rot={v_rot}")
        except (RpcError, ResponseError) as e:
            self.get_logger().error(f"Failed to send velocity command: {e}")

    def shutdown(self):
        """Shutdown the driver and release resources."""
        # power off requires lease so we do it before releasing
        if self.robot and self.robot.is_powered_on():
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            print("Robot powered off.")

        # Release the E-Stop.
        if self.estop_keep_alive:
            self.estop_keep_alive.shutdown()
            print("E-Stop released.")

        if self.lease_keep_alive:
            self.lease_keep_alive.shutdown()
            print("Lease released.")


def main(args=None):
    """Initialize and run the Spot ROS 2 driver node."""
    rclpy.init(args=args)
    spot_driver_node = None

    try:
        spot_driver_node = SpotROS2Driver()
        executor = MultiThreadedExecutor()
        executor.add_node(spot_driver_node)
        if rclpy.ok():
            executor.spin()
    except KeyboardInterrupt:
        if spot_driver_node:
            print("Shutting down the Robot due to KeyboardInterrupt.")
    except (RpcError, ResponseError, LeaseError) as e:
        if spot_driver_node:
            print(f"Shutting down the Robot due to Spot-SDK error: {e}")
    finally:
        if executor:
            executor.shutdown()
        if spot_driver_node:
            spot_driver_node.shutdown()
            spot_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
