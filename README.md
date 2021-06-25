# SLAM PROJECT 

Final project for Low Speed Vehicle SLAM Project.

Making map for ITI Library by differant slam algorithms.


- <img src="https://github.com/AlaaElnagar/ITI_SLAM/blob/main/img/slams.png" />




## **Project Goals**

The goal of this project was to make slam with different algorithms in ROS2 and test and review

the results and provide best configuration to get best result.

 For this one needed to:

- Download and build slam package.
- Work online or offline (means use bag file or connect to your robot).
- Make sure the transformation is correct.
- Change some configration in slam peackage.
- Run your package and visualize the result to rviz.
- Save your map

### 

### Prerequisites

(since I have not tested on multiple platforms, and versions, I am listing only the configuration I used)

- Ubuntu 20.04 OS 
- ROS2 Foxy
- following ROS packages were used and the process of obtaining them is detailed below:
  - [ROS2Foxy](https://docs.ros.org)
  - [Cartographer](https://github.com/ros2/cartographer)
  - [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)
  - [Rtapmap](https://github.com/introlab/rtabmap_ros/tree/ros2)
  - [Rviz](https://github.com/ros2/rviz)





## Cartographer



<<<<<<< HEAD
### Package Description

Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.  

## Getting start

you can download cartograoher Ros2 from this [Repo](https://github.com/ros2/cartographer).

create your workspace folder and inside this directory cerate src folder.

```
mkdir carto_ws/src -p
```

 Put that you download in src folder and change your directory back to carto_ws.

```
cd ..
```

start Build your Package.

```
colcon build --symlink-install
```





##  Build Package     
=======

## Slam Toolbox




## Rtabmap


>>>>>>> a567e8ade5c616e5bb705b488ced574082205538

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
    
    

# Slam_toolbox

## used pkgs

- ```slam_toolbox```   ```git clone -b foxy-devel https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel ``` 
  - note:
   -  don't forget to change the base_footprint into base_link in slam_toolbox config directory ```    base_frame: base_link ```
  
- ```robot_localization```   ```git clone -b https://github.com/cra-ros-pkg/robot_localization/tree/foxy-devel```
### Used commands 


- <img src="https://github.com/AlaaElnagar/ITI_SLAM/blob/main/img/slam_commands.png" />

### rqt_graph 

- <img src="https://github.com/AlaaElnagar/ITI_SLAM/blob/main/img/rosgraphv1.png" />




​    



