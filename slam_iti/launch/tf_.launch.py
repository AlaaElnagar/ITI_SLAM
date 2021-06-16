from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():   
#get parameters file 
    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    
#launch Lidar node  and odometery 
    return LaunchDescription([
    

#print frames            
        launch_ros.actions.Node(
            package='tf2_tools',
            executable='view_frames.py',
            output='screen',

            ),
            
#launch static transformation node        
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            ),
            
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            ),            
            
#launch robot localization to transform from base link to odom frame to map            
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),            
#local node to generate odom to base_link           
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]            #
           ),
#global_node to generate map to odom                        
        launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	    output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odom')]            #
           ),
           
	launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu/data', 'imu/data'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]
           )           

         

     
])
