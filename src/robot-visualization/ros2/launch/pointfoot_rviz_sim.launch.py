from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import sys

def generate_launch_description():
    robot_type = os.getenv("ROBOT_TYPE")
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)

    # URDF 文件路径（保持不变）
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "pointfoot/"+ robot_type +"/xacro", "robot.xacro"]
    )
    rviz_config_file = get_package_share_directory('robot_visualization') + '/rviz/pointfoot.rviz'

    # Load URDF
    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([

        # 保留 robot_state_publisher（必须）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        # 新增：joint_state_publisher_gui（带图形界面，可手动拖动滑块调整所有关节，超级适合仿真调试）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # 保留 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{"use_sim_time": True}],
        ),
    ])