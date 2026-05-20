from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim       = LaunchConfiguration('sim')
    use_manus = LaunchConfiguration('use_manus')
    use_pedal = LaunchConfiguration('use_pedal')
    use_vive  = LaunchConfiguration('use_vive')

    # In sim mode, hardware nodes are suppressed unless explicitly forced on.
    # use_pedal / use_vive default to 'true' but are overridden to 'false' by sim:=true
    # via the condition logic below (sim=true → not-sim=false → hardware disabled).
    not_sim = NotSubstitution(sim)

    pedal_on = AndSubstitution(use_pedal, not_sim)
    vive_on  = AndSubstitution(use_vive,  not_sim)

    return LaunchDescription([

        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Simulation mode: suppresses all hardware nodes (pedal, vive, manus)'),

        DeclareLaunchArgument(
            'use_manus',
            default_value='false',
            description='Launch Manus glove nodes (requires ManusSDK binary)'),

        DeclareLaunchArgument(
            'use_pedal',
            default_value='true',
            description='Launch PCsensor pedal node (requires physical pedal); ignored when sim:=true'),

        DeclareLaunchArgument(
            'use_vive',
            default_value='true',
            description='Launch Vive tracker node (requires SteamVR running); ignored when sim:=true'),

        # ── Input ──────────────────────────────────────────────────────────
        GroupAction(
            condition=IfCondition(pedal_on),
            actions=[
                Node(
                    package='pedal_ros2',
                    executable='pedal_node',
                    name='pedal_node',
                    output='screen',
                ),
            ],
        ),

        GroupAction(
            condition=IfCondition(vive_on),
            actions=[
                Node(
                    package='vive_ros2',
                    executable='vive_tracker_node',
                    name='vive_tracker_node',
                    output='screen',
                ),
            ],
        ),

        GroupAction(
            condition=IfCondition(AndSubstitution(use_manus, not_sim)),
            actions=[
                Node(
                    package='manus_ros2',
                    executable='manus_data_publisher',
                    name='manus_data_publisher',
                    output='screen',
                ),
                Node(
                    package='manus_inspire',
                    executable='manus_inspire_node',
                    output='screen',
                ),
            ],
        ),

        # ── Core ───────────────────────────────────────────────────────────
        Node(
            package='vive_rby1',
            executable='vive_rby1_node',
            name='vive_rby1_node',
            output='screen',
            parameters=[{'publish_rate': 100.0, 'ik_dt': 0.05, 'pos_scale': 0.5, 'torso_pos_scale': 1.0,
                         'tracker_smooth_alpha': 0.5}],
        ),

        # ── GUI ────────────────────────────────────────────────────────────
        Node(
            package='teleop_gui',
            executable='teleop_gui_node',
            name='teleop_gui',
            output='screen',
        ),

    ])
