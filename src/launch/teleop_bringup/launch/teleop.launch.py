from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Input ──────────────────────────────────────────────────────────
        Node(
            package='pedal_ros2',
            executable='pedal_node',
            name='pedal_node',
            output='screen',
        ),

        Node(
            package='vive_ros2',
            executable='vive_tracker_node',
            name='vive_tracker_node',
            output='screen',
        ),

        Node(
            package='manus_ros2',
            executable='manus_data_publisher',
            name='manus_data_publisher',
            output='screen',
        ),

        # ── Core ───────────────────────────────────────────────────────────
        Node(
            package='vive_rby1',
            executable='vive_rby1_node',
            name='vive_rby1_node',
            output='screen',
        ),

        Node(
            package='manus_inspire',
            executable='manus_inspire_node',
            name='manus_inspire_node',
            output='screen',
        ),

    ])
