import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Livox driver + RViz
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(livox_pkg, 'launch', 'rviz_MID360_launch.py')
            )
        )
    )

    # 2) Nodo cámara
    cam_pkg = get_package_share_directory('camera_driver')
    ld.add_action(
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'device_id': 0},
                {'width': 640},
                {'height': 480},
                {'fps': 30.0},
                {'camera_frame': 'camera_link'},
                {'calib_file': os.path.join(cam_pkg, 'calib', 'camera.yaml')}
            ]
        )
    )

    # 3) Nodo IMU
    ld.add_action(
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen',
        )
    )

    # 4) Nodo de fusión YOLO + LiDAR
    ld.add_action(
        Node(
            package='lidar_camera_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
        )
    )

    # 5) RViz para la fusión
    fusion_pkg = get_package_share_directory('lidar_camera_fusion')
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(fusion_pkg, 'rviz', 'fusion.rviz')]
        )
    )

    return ld

