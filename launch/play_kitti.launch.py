import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 実行している現在のユーザー名を取得
    user_name = os.getenv("USER")

    # 2. SSDのパスを構築
    defalt_dataset_path = f"/media/{user_name}/KIOXIA/opensource_slam_dataset/kitti/data_odometry_gray/dataset/sequences/00"

    # 3. launch引数の設定
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value=defalt_dataset_path,
        description='Path to the KITTI sequence folder'
    )

    # 4. ノードの設定
    kitti_node = Node(
        package='my_visual_slam',
        executable='slam_main',
        name='kitti_player_node',
        output='screen',
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'frame_rate': 10.0
        }]
    )

    return LaunchDescription([
        dataset_path_arg,
        kitti_node
    ])
