import time

# ROS 2 imports
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster 
from nav_msgs.msg import Odometry

# Boston Dynamics SDK imports
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

class SpotROS2Driver(Node):
    """A minimal ROS 2 driver for Boston Dynamics Spot robot."""

    def __init__(self):
        super().__init__('spot_driver_node')

        self.declare_parameter('hostname', '192.168.80.3')
        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        # TODO: Add parameter for robot username and password if needed
        self.robot = None
        self.lease_keep_alive = None

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