cmake_minimum_required(VERSION 3.5)
project(filter_pcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find system dependencies
#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(carkyo_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(tf2_geometry_msgs REQUIRED)
#find_package(tf2_ros REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  carkyo_msgs
  geometry_msgs
  tf2
  #tf2_geometry_msgs
  #tf2_ros
  #EIGEN3
  #PCL
)

include_directories(${PROJECT_NAME}/include
    include
    )

add_executable(filter_pcl_node src/filter_pcl.cpp)
ament_target_dependencies(filter_pcl_node rclcpp sensor_msgs)
target_link_libraries(filter_pcl_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS
  filter_pcl_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(close_obstacle_detection_node src/close_obstacle_detection.cpp)
ament_target_dependencies(close_obstacle_detection_node rclcpp sensor_msgs carkyo_msgs)  
target_link_libraries(close_obstacle_detection_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS
  close_obstacle_detection_node
  DESTINATION lib/${PROJECT_NAME})

  

  

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()



# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install the include directory  
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

find_package(rosidl_default_generators REQUIRED)


ament_package()
