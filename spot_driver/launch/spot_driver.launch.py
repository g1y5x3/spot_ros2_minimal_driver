"""Launch file for the Spot ROS2 Minimal Driver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for the Spot ROS2 driver."""
    # Declare launch arguments
    declare_hostname_arg = DeclareLaunchArgument(
        "hostname", default_value="192.168.80.3", description="IP address of the Spot robot"
    )

    declare_username_arg = DeclareLaunchArgument(
        "username", default_value="user", description="Username for the Spot robot"
    )

    declare_password_arg = DeclareLaunchArgument(
        "password", default_value="password", description="Password for the Spot robot"
    )

    declare_odomframe_arg = DeclareLaunchArgument(
        "odometry_frame", default_value="odom", description="Odometry frame to use (odom or vision)"
    )

    declare_streaming_client_arg = DeclareLaunchArgument(
        "use_streaming_client",
        default_value="false",
        description="Whether to use the streaming client (requires license)",
    )

    declare_rviz_arg = DeclareLaunchArgument("rviz", default_value="false", description="Whether to start RViz")

    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="spot.rviz", description="RViz configuration file name"
    )

    # Get launch configuration values
    hostname = LaunchConfiguration("hostname")
    username = LaunchConfiguration("username")
    password = LaunchConfiguration("password")

    odometry_frame = LaunchConfiguration("odometry_frame")
    use_streaming_client = LaunchConfiguration("use_streaming_client")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Spot driver node
    spot_driver_node = Node(
        package="spot_minimal_driver",
        executable="spot_driver_node",
        name="spot_driver_node",
        output="screen",
        parameters=[
            {
                "hostname": hostname,
                "username": username,
                "password": password,
                "odometry_frame": odometry_frame,
                "use_streaming_client": use_streaming_client,
            }
        ],
        sigterm_timeout=LaunchConfiguration("sigterm_timeout", default="30"),
        sigkill_timeout=LaunchConfiguration("sigkill_timeout", default="30"),
    )

    # RViz node with conditional launch
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", [FindPackageShare("spot_minimal_driver"), "/config/", rviz_config]],
    )

    # Group all nodes
    nodes_group = GroupAction(
        [
            spot_driver_node,
            rviz_node,
        ]
    )

    return LaunchDescription(
        [
            declare_hostname_arg,
            declare_username_arg,
            declare_password_arg,
            declare_odomframe_arg,
            declare_streaming_client_arg,
            declare_rviz_arg,
            declare_rviz_config_arg,
            nodes_group,
        ]
    )
