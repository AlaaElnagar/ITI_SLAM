# 2D Lidar slam 

- <img src="https://raw.githubusercontent.com/AlaaElnagar/ITI_SLAM/main/pic/slam.jpg" />

## Target 

- making map for ITI library using Lidar data and wheel odometery 
- Robot become able to localize itself 
- Ability of following the path through nav2 goal "rviz2 tool"


## Project phases  

# Embedded phase 

- Reading wheel odom data "Encoder" and passing it to serial monitor  
- Reading of heading data "IMU" and passing it using controller to serial monitor 
- Making Ethernet driver to connect our controller with jetson board or labtop and provide high speed data transfere 

# ROS Section 

- Ability of making high quality map as mentiond above 
- Reading lidar data using ```rplidar_ros2``` pkg
- Making ```py_serial``` pkg to recive all of our controller data in ROS 
-
-




