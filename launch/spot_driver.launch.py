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
        'hostname',
        default_value='192.168.80.3',
        description='IP address of the Spot robot'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='spot.rviz',
        description='RViz configuration file name'
    )

    # Get launch configuration values
    hostname = LaunchConfiguration('hostname')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Spot driver node
    spot_driver_node = Node(
        package='spot_minimal_driver',
        executable='spot_driver_node',
        name='spot_driver_node',
        output='screen',
        parameters=[{
            'hostname': hostname,
        }],
    )

    # RViz node with conditional launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', [
            FindPackageShare('spot_minimal_driver'),
            '/config/',
            rviz_config
        ]]
    )

    # Group all nodes
    nodes_group = GroupAction([
        spot_driver_node,
        rviz_node,
    ])

    return LaunchDescription([
        declare_hostname_arg,
        declare_use_rviz_arg,
        declare_rviz_config_arg,
        nodes_group,
    ])
