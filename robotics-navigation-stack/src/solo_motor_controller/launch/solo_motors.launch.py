from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    right = Node(
        package='solo_motor_controller',
        executable='right_motor_node',
        name='right_motor_node',
        parameters=[{
            'motor_port': '/dev/solo_right',
            'motor_addr': 1,
            'loop_rate': 50.0,
            'encoder_topic': 'right_encoder_counts',
            'speed_cmd_topic': 'right_motor_speed_cmd',
        }],
        output='screen',
    )

    left = Node(
        package='solo_motor_controller',
        executable='left_motor_node',
        name='left_motor_node',
        parameters=[{
            'motor_port': '/dev/solo_left',
            'motor_addr': 2,
            'loop_rate': 50.0,
            'encoder_topic': 'left_encoder_counts',
            'speed_cmd_topic': 'left_motor_speed_cmd',
        }],
        output='screen',
    )

    return LaunchDescription([right, left])


