Tabel of content

[TOC]



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

## start Build your Package.

```
colcon build --symlink-install
```

## now ,show time...

we have a bag file contains lider laser-scan and odometry/wheel msg for [ITI LIBRARY](https://www.iti.gov.eg/iti/home).

note:
you should using fresh time stamp frames if you use bag files.
use this pkg to refesh your time stamp [fresh time stamp pkg](https://github.com/SaeedMohamedd/Refresh-Time-Stamp-in-Bag-file).



first transformation:

1 static transformation between baselink and laser frame

```
ros2 run tf2_ros static_transform_publisher 0. 0. 0. 0. 0. 0. base_link laser
```

2. tranformation from map to odom using robot localization package.

```
ros2 launch robot_localization  dual_ekf_navsat_example.launch.py 
```



second run cartographer node and show map in rviz

```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

third save the map using map saver 

```
ros2 run nav2_map_server map_saver_cli -f /home/saeed/maps/iti_library/map
```

the output should be two files  map.yaml and map.pgm

![](/img/cartographer_map.png)



## Cartographer Drawbacks

1. for this time that i write this artical there are  137 open issue you can track the progress of this issue using this link [cartographer_ros_issue](https://github.com/cartographer-project/cartographer_ros/issues).
2. the documentation and tutorial are avilabile in ros1 only .
3. not working well on ros2 foxy.



### SO, we move to SLAM ToolBox.



## Slam Toolbox

###  introduction

Slam Toolbox is a set of tools and capabilities for 2D SLAM built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101) while at [Simbe Robotics](https://www.simberobotics.com/), maintained whil at Samsung Research, and largely in his free time.

This project contains the ability to do most everything any other  available SLAM library, both free and paid, and more. This includes:

- Ordinary point-and-shoot 2D SLAM mobile robotics folks expect  (start, map, save pgm file) with some nice built in utilities like  saving maps
- Continuing to refine, remap, or continue mapping a saved (serialized) pose-graph at any time
- life-long mapping: load a saved pose-graph continue mapping in a  space while also removing extraneous information from newly added scans
- an optimization-based localization mode built on the pose-graph.  Optionally run localization mode without a prior map for "lidar  odometry" mode with local loop closures
- synchronous and asynchronous modes of mapping
- kinematic map merging (with an elastic graph manipulation merging technique in the works)
- plugin-based optimization solvers with a new optimized Google Ceres based plugin
- RVIZ plugin for interacting with the tools
- graph manipulation tools in RVIZ to manipulate nodes and connections during mapping
- Map serialization and lossless data storage
- ... more but those are the highlights.

For more details on this [Slam Toolbox Repo](https://github.com/SteveMacenski/slam_toolbox) .

## Getting start

you can download slam toolbox Ros2 from this [Slam Toolbox Repo](https://github.com/SteveMacenski/slam_toolbox) .

make sure that you download the right branch from the repo,in my case i choose foxy-devel branch. 

create your workspace folder and inside this directory cerate src folder.

```
mkdir slam_toolbox_ws/src -p
```

 Put that you download in src folder and change your directory back to slam_toolbox_ws.

```
cd ..
```

## start Build your Package.

```
colcon build --symlink-install
```

## now ,show time...

we have a bag file contains lider laser-scan and odometry/wheel msg for [ITI LIBRARY](https://www.iti.gov.eg/iti/home).

note:
you should using fresh time stamp frames if you use bag files.
use this pkg to refesh your time stamp [fresh time stamp pkg](https://github.com/SaeedMohamedd/Refresh-Time-Stamp-in-Bag-file).



first transformation:

1 static transformation between baselink and laser frame

```
ros2 run tf2_ros static_transform_publisher 0. 0. 0. 0. 0. 0. base_link laser
```

2. tranformation from map to odom using robot localization package.

```
ros2 launch robot_localization  dual_ekf_navsat_example.launch.py 
```

ه Important notes  :

in slam_tool

3. run slam toolbox node:
   - offline_launch.py                                   ====>     using bag file.
   - online_sync_launch.py                          ====>    using high computational power
   - online_async_launch.py                        ====>  limited computational power
   - lifelong_launch.py                                  ====> complete mapping on old map.
   - localization_launch.py                           ====> localization mode
   - merge_maps_kinematic_launch.py     ====> merge maps


















## Rtabmap



















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



