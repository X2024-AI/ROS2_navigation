from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')

    # Include PAL's TIAGo simulation DIRECTLY (br2_tiago's sim.launch.py adds
    # nothing but does NOT forward the gzclient argument, which we need:
    # on this hybrid-NVIDIA host a live gzclient can starve gzserver's sensor
    # rendering - the laser never initializes. Headless by default; pass
    # gzclient:=true (or KEEP_GUI=1 via bringup_m6.sh) to watch the GUI.
    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={
            'world_name': 'small_office_inventory',
            'arm': 'no-arm',
            'is_public_sim': 'True',
            'gzclient': LaunchConfiguration('gzclient'),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch the Gazebo GUI (True = watch the robot; use HEADLESS=1 via bringup for benchmark/degraded-session runs)'),
        include_sim,
        tuck_retry,
    ])
