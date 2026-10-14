from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    br2_tiago_dir = get_package_share_directory('br2_tiago')

    # Reuse the book's TIAGo simulation, but pin the warehouse world
    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(br2_tiago_dir, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': 'small_office_inventory',
            'is_public_sim': 'True',
        }.items(),
    )

    # The built-in tuck races play_motion2 startup under boot contention;
    # retry patiently so the arm is always tucked in the end.
    tuck_retry = Node(
        package='wr_bringup',
        executable='tuck_arm_retry.sh',
        name='tuck_arm_retry',
        output='screen',
    )

    return LaunchDescription([include_sim, tuck_retry])
