from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_for_pick_place_voice',
            executable='get_keyword_client',
            name='get_keyword_client',
            output='screen'
        ),
        Node(
            package='llm_for_pick_place_voice',
            executable='get_keyword',
            name='get_keyword',
            output='screen'
        ),
        Node(
            package='llm_for_pick_place_voice',
            executable='pick_rl_node',
            name='pick_rl_node',
            output='screen'
        ),
        Node(
            package='llm_for_pick_place_voice',
            executable='place_rl_node',
            name='place_rl_node',
            output='screen'
        ),
    ])
