```
ros2 launch spot_minimal_driver spot_driver.launch.py
```

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

```
ros2 action send_goal /navigate_to spot_action/action/NavigateTo "{x: 1.0, y: 0.0, yaw: 0.0}"
```

```
ros2 service call /get_fiducial_transform spot_srvs/srv/GetTransform "{}"
```

```
ros2 run tf2_ros static_transform_publisher -0.10795 0.0 0.1397 1.57 0 0 world filtered_fiducial_200
```

```
ros2 run map_localization map_localizer_node
```
