import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rdda_interface'),
        'config/rdda_params.yaml'
    ])

    anti_alias_freq = LaunchConfiguration('anti_alias_freq', default='20.0')
    max_vel = LaunchConfiguration('max_vel', default='3.0')
    max_eff = LaunchConfiguration('max_eff', default='5.0')
    stiff = LaunchConfiguration('stiff', default='0.0')
    home = LaunchConfiguration('home', default='0')
    output = LaunchConfiguration('output', default='screen')
    rdda_type = LaunchConfiguration('rdda_type', default='right_gripper')

    return launch.LaunchDescription([
        DeclareLaunchArgument('anti_alias_freq', default_value='20.0'),
        DeclareLaunchArgument('max_vel', default_value='3.0'),
        DeclareLaunchArgument('max_eff', default_value='5.0'),
        DeclareLaunchArgument('stiff', default_value='0.0'),
        DeclareLaunchArgument('home', default_value='0'),
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('config_file', default_value=config_file_path),

        # Set parameters
        Node(
            package='rdda_interface',
            executable='rdda_interface_node',
            name=[
                launch.substitutions.TextSubstitution(text='rdda_interface_'),
                rdda_type
            ],            output=output,
            parameters=[config_file_path, {
                'anti_alias_freq': anti_alias_freq,
                'max_vel': max_vel,
                'max_eff': max_eff,
                'stiff': stiff
            }],
            arguments=[rdda_type]
        ),

        # Conditional homing group
        GroupAction([
            OpaqueFunction(
                function=lambda context: [
                    Node(
                        package='rdda_interface',
                        executable='homing.py',
                        name='homing',
                        arguments=[home]
                    )
                ]
            )
        ], condition=IfCondition(home))
    ])
