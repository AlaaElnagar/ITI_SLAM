from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="filter_pcl",
            executable="filter_pcl_node",
            name="filter_pcl_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pcl_boundaries": "-0.3 0.3 -0.2 0.1 0 0.5"},
#                {"pcl_boundaries": "-0.1 .1 -1 0.1 0 0.7"},
                {"ip_Topic_name": "/intel_realsense_d435_depth/points"},
                {"op_Topic_name": "/intel/cropped"},
            ]
           ),      
        Node(
            package="filter_pcl",
            executable="close_obstacle_detection_node",
            name="close_obstacle_detection_node",
            output="screen",
            emulate_tty=True,

           ) 
        
])
    
# "pcl_boundaries": "-1.1 1.1 -1 0.1 0 0.7" left right up down close_forward far_forward
# a b c d e f
# a, b left , right
#c, d up & down
#e f forward (e = 0 is camera f = x as x is max required distance)
