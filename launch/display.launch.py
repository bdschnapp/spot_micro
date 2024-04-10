import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_cfg_path = os.path.realpath(get_package_share_directory('spot_micro_robot'))
    rviz_path=robot_cfg_path+'/config/config.rviz'

    urdf = os.path.join(
        get_package_share_directory('spot_micro_robot'),
        'urdf/urdfspot_micro_urdf.urdf'
        )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
          package='rviz2', executable='rviz2', name="rviz2", output='screen', arguments=['-d'+str(rviz_path)]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),

        Node(
           package="joint_state_publisher_gui",
           executable="joint_state_publisher_gui",
           name="joint_state_publisher_gui",
           output="screen"
        )
    ])



