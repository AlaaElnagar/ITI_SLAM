from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    
    return LaunchDescription([


        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['-0.032', '0', '0.171', '0', '0', '0', 'hansa_magni::base_link', 'hansa_magni::base_scan'],
            ),

        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0.07', '0', '0.13', '0', '0', '0', 'hansa_magni::base_link', 'hansa_magni::front_cam::camera_depth_optical_frame'],
            ),


        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.1', '0', '0', '0', 'hansa_magni::base_footprint', 'hansa_magni::base_link'],
            ),
                                    
        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
                                    

    ])
