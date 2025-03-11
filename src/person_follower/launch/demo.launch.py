import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    package_name = 'person_follower'
    package_share_dir = get_package_share_directory(package_name)

    # Default RViz config file path
    default_rviz_config = os.path.join(package_share_dir, 'config', 'rviz_config.rviz')

    # Declare RViz config file argument
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz configuration file'
    )

    # Load the RViz config file
    rviz_config_file = LaunchConfiguration('rviz_config')

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file],
        remappings=[
            ('/tf', '/a200_0000/tf'),
            ('/tf_static', '/a200_0000/tf_static')
        ]
    )

    # Person Detection Node
    person_detection_node = Node(
        package='person_follower',
        executable='person_detection',
        name='person_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', '/a200_0000/tf'),
            ('/tf_static', '/a200_0000/tf_static')
        ]
    )

    # Include the Clearpath Gazebo Simulation Launch File
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clearpath_gz'),
                'launch/simulation.launch.py'
            )
        )
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        person_detection_node,
        simulation_launch
    ])
