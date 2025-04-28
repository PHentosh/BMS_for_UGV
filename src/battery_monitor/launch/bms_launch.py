from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_husky_battery = PathJoinSubstitution(
        [FindPackageShare('battery_monitor'),
        'cfg',
        'params.yaml'],
    )

    last_battery_state = PathJoinSubstitution(
        [FindPackageShare('battery_monitor'),
        'cfg',
        'state.yaml'],
    )
    

    return LaunchDescription([
        Node(
            package='battery_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'battery_general_params': config_husky_battery,
                 "battery_state_params":   last_battery_state}
            ]
        )
    ])
