from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import sys

def generate_launch_description():
    robot_type = os.getenv("ROBOT_TYPE")

    #设置雷达启动节点
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10.yaml')
                     
    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',		#设置激光数据topic名称
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )

    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)
        
    rl_type = os.getenv("RL_TYPE")
    if not rl_type:
        print("\033[31mError: Please set the RL_TYPE using 'export RL_TYPE=isaacgym/isaaclab'.\033[0m")
        sys.exit(1)

    robot_controllers_encoder_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/"+ robot_type +"/policy/" + rl_type + "/encoder.onnx"])
    robot_controllers_policy_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/"+ robot_type +"/policy/" + rl_type + "/policy.onnx"])
    robot_controllers_pointfoot_params_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/pointfoot/"+ robot_type +"/params.yaml"])
    robot_controllers_file = PathJoinSubstitution([FindPackageShare("robot_controllers"), "config/robot_controllers.yaml"])
    robot_hw_joystick_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/joystick.yaml"])
    robot_hw_file = PathJoinSubstitution([FindPackageShare("robot_hw"), "config/robot_hw.yaml"])

    return LaunchDescription([
        Node(
            package='robot_hw',
            executable='pointfoot_node',
            name='robot_hw_node',
            output='screen',
            arguments=["10.192.1.2"],
            parameters=[
                {
                    "robot_controllers_encoder_file": robot_controllers_encoder_file,
                    "robot_controllers_policy_file": robot_controllers_policy_file,
                },
                robot_controllers_pointfoot_params_file, 
                robot_controllers_file,
                robot_hw_joystick_file,
                robot_hw_file,
            ],
        ),

        #雷达启动节点
        driver_node,

        #雷达数据补偿节点
        Node(
            package='lidar_odometry',
            executable='scan_corrector_node',
            output='screen',
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            output='screen',
            parameters=[{"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'true'
            }.items()
        ),
    ])