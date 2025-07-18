import time

# ROS 2 imports
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist

# Boston Dynamics SDK imports
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b


class SpotROS2Driver(Node):
    """A minimal ROS 2 driver for Boston Dynamics Spot robot."""

    def __init__(self):
        super().__init__('spot_driver_node')

        self.declare_parameter('hostname', '192.168.80.3')
        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        # TODO: Add parameter for robot username and password if needed
        self.robot = None
        self.lease_keep_alive = None

        # ROS 2 publishers and subscribers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        try:
            # Robot initialization 
            sdk = bosdyn.client.create_standard_sdk('SpotROS2DriverClient')
            self.robot = sdk.create_robot(self.hostname)
            bosdyn.client.util.authenticate(self.robot)
            self.robot.time_sync.wait_for_sync()

            assert not self.robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                             'such as the estop SDK example, to configure E-Stop.'

            self.get_logger().info(f'Successfully authenticated and connected to the robot.')   

            # Create SDK clients
            self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
            self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
            self.get_logger().info('Robot clients created.')

            # Lease management
            self.lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
            self.get_logger().info('Acquired lease.')

            # Power on and Stand Robot
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), 'Robot power on failed.'
            self.get_logger().info('Robot powered on.')

            blocking_stand(self.command_client, timeout_sec=10)
            self.get_logger().info('Robot standing.')
            time.sleep(3)

        except Exception as e:
            self.get_logger().error(f'Failed to connect to the robot: {e}')
            raise

        # Main Loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    
    def timer_callback(self):
        """Periodic publish robot data (if connected)."""
        robot_state = self.robot_state_client.get_robot_state()
        odom_tfrom_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, 
                                        ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        self.publish_transform(odom_tfrom_body)

    def publish_transform(self, odom_tfrom_body):
        """Publish the transform from ODOM to BODY frame."""
        t = TransformStamped()
        # TODO: sync with the robot's internal time
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_tfrom_body.position.x
        t.transform.translation.y = odom_tfrom_body.position.y
        t.transform.translation.z = odom_tfrom_body.position.z
        t.transform.rotation.x = odom_tfrom_body.rotation.x
        t.transform.rotation.y = odom_tfrom_body.rotation.y
        t.transform.rotation.z = odom_tfrom_body.rotation.z
        t.transform.rotation.w = odom_tfrom_body.rotation.w

        self.tf_broadcaster.sendTransform(t)     

    def cmd_vel_callback(self, msg: Twist):
        """Callback for the /cmd_vel topic."""
        v_x = msg.linear.x
        v_y = msg.linear.y
        v_rot = msg.angular.z

        # Create a velocity command
        command = RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot)
        end_time = time.time() + 0.6
        
        try:
            # Send the command to the robot
            self.command_client.robot_command(command, end_time_secs=end_time)
            self.get_logger().debug(f'Sent velocity command: v_x={v_x}, v_y={v_y}, v_rot={v_rot}')
        except Exception as e:
            self.get_logger().error(f'Failed to send velocity command: {e}')
    
    def shutdown(self):
        """Shutdown the driver and release resources."""
        # power off requires lease so we do it before releasing
        if self.robot and self.robot.is_powered_on():
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            print('Robot powered off.')

        if self.lease_keep_alive:
            self.lease_keep_alive.shutdown()
            print('Lease released.')


def main(args=None):
    rclpy.init(args=args)
    spot_driver_node = None

    try:
        spot_driver_node = SpotROS2Driver()
        # Only spin if the node was successfully created (no exceptions in __init__)
        if rclpy.ok():
            rclpy.spin(spot_driver_node)
    except (KeyboardInterrupt, Exception) as e:
        if spot_driver_node:
            # ros2 context could be exited by this point, use get_logger() might fail
            print(f'Shutting down the Robot due to {type(e).__name__}.')
    finally:
        if spot_driver_node:
            spot_driver_node.shutdown()
            spot_driver_node.destroy_node()

if __name__ == '__main__':
    main()