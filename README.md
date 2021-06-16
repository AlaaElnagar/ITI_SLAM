# 2D Lidar slam 

- <img src="https://github.com/AlaaElnagar/ITI_SLAM/blob/main/img/iti_liberary%20_map_v1.0.png" />

## Target 

- making map for ITI library using Lidar data and wheel odometery 
- Robot become able to localize itself 
- Ability of following the path through nav2 goal "rviz2 tool"


## Project phases  

# Embedded phase 

- Reading wheel odom data "Encoder" and passing it to serial monitor  
- Reading of heading data "IMU" and passing it using controller to serial monitor 
- Making Ethernet driver to connect our controller with jetson board or labtop and provide high speed data transfere 

# ROS phases 

- Ability of making high quality map as mentiond above 
- localize a robot in an environment
- perform path planning from an initial position to a goal destination
- avoid obstacles while executing the planned path
- Reading lidar data using ```rplidar_ros2``` pkg
- Making ```py_serial``` pkg to recive all of our controller data in ROS 

# We can use
- ```Cartographer``` pkg to build the environment map and then providing the map to a ```map_server```
-  ```AMCL``` pkg to give robot accurate localization 
- ```dual_ekf``` to fuse "IMU","Wheel_odometry" and lidar to get accurate estimated pose 

# Today  Progress 10 jun 2021
- Runing ```Robotlocalization``` pkg 
- Static transformation for laser using ```tf2_ros_static_transform_publisher``` using ```ros2 run tf2_ros static_transform_publisher 0. 0. 0. 0. 0. 0. base_link laser```
- Map generation using ```cartographer```  
- map deployment into ```nav2_bringup```
## Result 
  It works !
# Next steps
-impelementation of all of the above in real robot 

## Real robot requirements 
1- Receiving 
    Odometry data from controller 
    Laser data from ```rplidar_ros2``` pkg
    
  



