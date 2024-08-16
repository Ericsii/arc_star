from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('arc_star')
    default_config_file = PathJoinSubstitution(
        [package_path, 'config', 'params.yaml'])

    config_file = LaunchConfiguration(
        'config_file', default=default_config_file)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the config file to use')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    arc_star_node = Node(
        package='arc_star',
        executable='arc_star_node',
        name='arc_star_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(arc_star_node)

    return ld
