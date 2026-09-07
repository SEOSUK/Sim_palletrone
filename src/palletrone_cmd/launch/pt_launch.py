from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('palletrone_interfaces'), 'config')
    parameters = {
        'control_config': LaunchConfiguration('control_config'),
        'model_config': LaunchConfiguration('model_config'),
    }
    command_parameters = {
        'control_config': LaunchConfiguration('control_config'),
        'command_config': LaunchConfiguration('command_config'),
        'model_config': LaunchConfiguration('model_config'),
    }
    visualization_dir = get_package_share_directory('palletrone_visualization')
    with open(os.path.join(visualization_dir, 'urdf', 'palletrone.urdf'), encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    plant = Node(
        package='plant', executable='plant', name='plant', output='screen',
        parameters=[parameters, {'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool)}],
    )
    wrench = Node(package='palletrone_controller', executable='wrench_controller',
                  name='wrench_controller', output='screen', parameters=[parameters])
    allocator = Node(package='palletrone_controller', executable='allocator_controller',
                     name='allocator_controller', output='screen', parameters=[parameters])
    command = Node(package='palletrone_cmd', executable='command_node',
                   name='command_node', output='screen', parameters=[command_parameters],
                   condition=IfCondition(LaunchConfiguration('command')))
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('visualization')))
    visualization = Node(
        package='palletrone_visualization', executable='palletrone_visualization_node',
        name='palletrone_visualization', output='screen', parameters=[command_parameters],
        condition=IfCondition(LaunchConfiguration('visualization')))
    logger = Node(
        package='palletrone_interfaces', executable='csv_logger', name='csv_logger', output='screen',
        parameters=[{'command_config': LaunchConfiguration('command_config'),
                     'output_directory': LaunchConfiguration('log_directory')}],
        condition=IfCondition(LaunchConfiguration('logging')))
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(visualization_dir, 'rviz', 'palletrone.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')))
    actions = [
        DeclareLaunchArgument('control_config', default_value=os.path.join(config_dir, 'control.yaml')),
        DeclareLaunchArgument('model_config', default_value=os.path.join(config_dir, 'model.yaml')),
        DeclareLaunchArgument('command_config', default_value=os.path.join(config_dir, 'command.yaml')),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('command', default_value='false',
                             description='Enable the built-in /cmd publisher for /command/key input; '
                                         'leave false when running command_node in a keyboard terminal'),
        DeclareLaunchArgument('visualization', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('logging', default_value='true'),
        DeclareLaunchArgument('log_directory',
                              default_value='',
                              description='CSV directory; empty uses the source '
                                          'palletrone_interfaces/bag directory'),
    ]
    for node in (plant, wrench, allocator, command, logger):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(reason='Palletrone node exited'))])))
    return LaunchDescription(actions + [plant, wrench, allocator, command,
                                        robot_state_publisher, visualization, logger, rviz])
