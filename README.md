```
ros2 launch spot_minimal_driver spot_driver.launch.py
```

```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

```
ros2 action send_goal /navigate_to spot_action/action/NavigateTo "{x: 1.0, y: 0.0, yaw: 0.0}"
```
