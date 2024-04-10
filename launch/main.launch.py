import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    robot_cfg_path = os.path.realpath(get_package_share_directory('spot_micro_robot'))
    rviz_path=robot_cfg_path+'/config/config.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('use_rviz', default='false')

    urdf = os.path.join(
        get_package_share_directory('spot_micro_robot'),
        'urdf/urdfspot_micro_urdf.urdf'
        )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    launch_argument = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf])
    
    spot_micro_gpio_node = Node(
            package='spot_micro_gpio',
            executable='main',
            name='SpotMicroGpio',
            output='screen',
            parameters=['./spot_micro_gpio_cfg.yaml']
    )

    spot_micro_ctrl_node = Node(
            package='spot_micro_ctrl',
            executable='main',
            name='spot_micro_ctrl',
            output='screen',
            parameters=['./spot_micro_ctrl_cfg.yaml']
    )
    
    '''
    # joint state publisher
    joint_state_publisher_node = Node(
            package='spot_micro_robot',
            executable='state_publisher',
            name='state_publisher',
            output='screen')
    ''' 

    launch_description = LaunchDescription([
        launch_argument,
        robot_state_publisher_node
        # joint_state_publisher_node,
    ])

    if launch_rviz:
        launch_description.add_action(
            Node(
                package='rviz2', executable='rviz2', name="rviz2", output='screen', arguments=['-d'+str(rviz_path)]
            )
        )
