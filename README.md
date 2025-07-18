```
ros2 launch spot_minimal_driver spot_driver.launch.py
```

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```