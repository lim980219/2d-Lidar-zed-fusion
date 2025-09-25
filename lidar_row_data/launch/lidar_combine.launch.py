from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a3_launch.py'
            )
        )
    )

    row_publisher = Node(
        package='lidar_row_data',
        executable='lidar_row_data',
        name='lidar_row_data',
        output='screen'
    )


    return LaunchDescription([
        lidar_launch,
        row_publisher,
        
    ])