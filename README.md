## To launch the driver
```
ros2 launch spot_minimal_driver spot_driver.launch.py
```

```
ros2 run tf2_ros static_transform_publisher -0.10795 0.0 0.1397 -1.57 -1.57 0 world wall
```

```
ros2 service call /get_fiducial_transform spot_srvs/srv/GetTransform "{fiducial_name: 'wall'}"
```

```
ros2 run map_localization map_localizer_node
```

```
ros2 run nav_goal_listener nav_goal_listener
```

## To control spot
```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

```
ros2 action send_goal /move_relative_xy spot_action/action/MoveRelativeXY "{x: 1.0, y: 0.0, yaw: 90.0}"
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Setup the docker container on Spot CORE I/O
sudo docker build -t spot_coreIO_ros2 --platform linux/arm64 -f Dockerfile.l4t .