from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():    
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '1.77', '0', '0', '0', 'base_link', 'laser'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            ),          

     

    ])
