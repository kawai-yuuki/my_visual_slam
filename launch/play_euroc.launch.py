import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 実行している現在のユーザー名を取得
    user_name = os.getenv("USER")

    # 2. SSDのパスを構築
    defalt_dataset_path = f"/media/{user_name}/KIOXIA/opensource_slam_dataset/euroc/machine_hall"

    # 3. launch引数の設定
    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='MH_01_easy',
        description='EuRoC sequence name'
    )

    dataset_path_expr = PythonExpression([
        "'", defalt_dataset_path, "/' + '", LaunchConfiguration('sequence'), "' + '/' + '", LaunchConfiguration('sequence'), "'"
    ])

    # 4. ノードの設定
    euroc_node = Node(
        package='my_visual_slam',
        executable='euroc_player',
        name='euroc_player_node',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path_expr,
            'publish_rate': 20.0
        }]
    )

    return LaunchDescription([
        sequence_arg,
        euroc_node
    ])
