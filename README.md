```
ros2 launch spot_minimal_driver spot_driver.launch.py
```

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

```
ros2 action send_goal /move_relative_xy spot_action/action/MoveRelativeXY "{x: 1.0, y: 0.0, yaw: 90.0}"
```
