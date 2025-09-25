from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 런치 인자 선언: camera_model (필수)
    camera_model = LaunchConfiguration('camera_model')
    declare_camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        description=(
            "[REQUIRED] The model of the camera. Valid choices are: "
            "['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']"
        )
    )

    # zed_camera.launch.py 경로
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': camera_model,
        }.items()
    )

    # zed_row_image 노드 실행
    row_publisher = Node(
        package='zed_row_video',
        executable='zed_row_video',  # setup.py의 entry point 이름
        name='zed_row_image_node',
        output='screen'
    )

    return LaunchDescription([
        declare_camera_model_arg,
        zed_launch,
        row_publisher
    ])