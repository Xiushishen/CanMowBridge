**CanMowBridge**

CanMowBridge is an innovative open-source project designed to enhance the autonomy and intelligence of lawn maintenance. By integrating NVIDIA Jetson Orin with Greenworks ZTR mowers through the CANbus interface and Robot Operating System (ROS), CanMowBridge enables precise control and communication of autonomous mowing operations.

To start node for interaction:
```
roslaunch can_ros1_driver can_control.launch
```

To collect data from Wheel Encoder, IMU, and GPS:
```
roslaunch can_ros1_driver optimus.launch
```
