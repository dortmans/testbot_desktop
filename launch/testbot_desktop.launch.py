from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, LogInfo, SetEnvironmentVariable, ExecuteProcess)
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, FindExecutable)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import (Node, PushRosNamespace, SetParameter)

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    ),
    DeclareLaunchArgument(
        'ip',
        default_value='',
        description='Robot IPaddress'
    ),
    DeclareLaunchArgument(
        'domain',
        default_value='0',
        description='ROS_DOMAIN_ID'
    ),
    DeclareLaunchArgument(
        'protocol',
        default_value='udp',
        description='Zenoh Bridge Protocol (udp or tcp)'
    ),
    DeclareLaunchArgument(
        'mode',
        default_value='peer',
        description='Zenoh Bridge Mode (peer or client)'
    ),
]


def generate_launch_description():

    zenoh_bridge_config_path = PathJoinSubstitution(
        [FindPackageShare('testbot_bringup'), 'config',
         'zenoh-bridge-ros2dds.json5']
    )

    rviz2_config = PathJoinSubstitution(
        [FindPackageShare('testbot_desktop'), 'config', 'testbot.rviz'])

    log_launching = LogInfo(msg="Launching Testbot Desktop")

    set_rmw_implementation = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')

    set_cyclonedds_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=PathJoinSubstitution(
            [FindPackageShare('testbot_desktop'), 'config', 'cyclonedds.xml']))

    set_ros_local_host_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY', value='0')

    set_ros_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID', value=LaunchConfiguration('domain'))

    set_use_sim_time = SetParameter(
        name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    launch_zenoh_bridge = ExecuteProcess(
        cmd=[FindExecutable(name='zenoh-bridge-ros2dds'),
             '-c',
             zenoh_bridge_config_path,
             '-m',
             LaunchConfiguration('mode'),
             '-e',
             [LaunchConfiguration('protocol'), '/',
              LaunchConfiguration('ip'), ':7447'],
             '--no-multicast-scouting'
             ],
        shell=True
    )

    launch_desktop_nodes = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             remappings=[
                 ('/tf', 'tf'),
                 ('/tf_static', 'tf_static')
             ],
             output='screen'
             ),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(log_launching)
    ld.add_action(set_rmw_implementation)
    ld.add_action(set_cyclonedds_uri)
    ld.add_action(set_ros_local_host_only)
    ld.add_action(set_ros_domain_id)
    ld.add_action(set_use_sim_time)
    ld.add_action(launch_zenoh_bridge)
    ld.add_action(launch_desktop_nodes)

    return ld
