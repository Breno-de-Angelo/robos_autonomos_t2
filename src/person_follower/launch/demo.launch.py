import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    package_name = 'person_follower'
    robot_name = 'a200_0000'
    package_share_dir = get_package_share_directory(package_name)

    # Default RViz config file path
    default_rviz_config = os.path.join(package_share_dir, 'config', 'rviz_config.rviz')
    explore_lite_launch = os.path.join(
        get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py'
    )

    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz configuration file'
    )

    clearpath_dir_arg = DeclareLaunchArgument(
        'clearpath_dir_path',
        default_value=os.getenv('HOME') + '/clearpath/',
        description='Base directory for Clearpath setup'
    )

    # Load launch configurations
    rviz_config_file = LaunchConfiguration('rviz_config')
    clearpath_dir_path = LaunchConfiguration('clearpath_dir_path')

    # RViz Node
    rviz = GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file],
            remappings=[
                ('/tf', f'/{robot_name}/tf'),
                ('/tf_static', f'/{robot_name}/tf_static')
            ]
        )
    ])

    # Person Detection Node
    person_detection_node = Node(
        package=package_name,
        executable='person_detection',
        name='person_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', f'/{robot_name}/tf'),
            ('/tf_static', f'/{robot_name}/tf_static')
        ]
    )

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clearpath_nav2_demos'),
                'launch/slam.launch.py'
            )
        ),
        launch_arguments={
            'setup_path': clearpath_dir_path,
            'use_sim_time': 'true'
        }.items()
    )

    nav2 = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap('/' + robot_name + '/global_costmap/sensors/lidar2d_0/scan',
                 '/' + robot_name + '/sensors/lidar2d_0/scan'),
        SetRemap('/' + robot_name + '/local_costmap/sensors/lidar2d_0/scan',
                 '/' + robot_name + '/sensors/lidar2d_0/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('params_file', f"{package_share_dir}/config/nav2_params.yaml"),
                ('use_composition', 'False'),
                ('namespace', robot_name)
              ]
        ),
    ])

    explore_lite = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap('/' + robot_name + '/goal_pose',
                 '/' + robot_name + '/exploration_goal_pose'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([explore_lite_launch]),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        )
    ])
    
    nav2_mux = GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package=package_name,
            executable='nav2_mux',
            name='nav2_mux',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/tf', f'/{robot_name}/tf'),
                ('/tf_static', f'/{robot_name}/tf_static')
            ]
        )
    ])

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
        clearpath_dir_arg,
        rviz,
        person_detection_node,
        slam,
        nav2,
        explore_lite,
        nav2_mux,
        simulation_launch
    ])
