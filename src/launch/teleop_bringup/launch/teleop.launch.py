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
    urdf_path = LaunchConfiguration('urdf_path')
    srdf_path = LaunchConfiguration('srdf_path')

    not_sim   = NotSubstitution(sim)
    pedal_on  = AndSubstitution(use_pedal, not_sim)
    vive_on   = AndSubstitution(use_vive,  not_sim)
    manus_on  = AndSubstitution(use_manus, not_sim)

    return LaunchDescription([

        # ── Launch arguments ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'urdf_path',
            default_value='',
            description='Absolute path to rby1.urdf (required for IK; pass via urdf_path:=<path>)'),

        DeclareLaunchArgument(
            'srdf_path',
            default_value='',
            description='Absolute path to rby1.srdf (pass via srdf_path:=<path>)'),

        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Simulation mode: suppress all hardware nodes (pedal, vive, manus)'),

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

        # ── Input: pedal ───────────────────────────────────────────────────
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

        # ── Input: Vive tracker ────────────────────────────────────────────
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

        # ── Input: Manus gloves ────────────────────────────────────────────
        GroupAction(
            condition=IfCondition(manus_on),
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
                    name='manus_inspire',
                    output='screen',
                ),
            ],
        ),

        # ── Core: teleop + IK ─────────────────────────────────────────────
        Node(
            package='vive_rby1',
            executable='vive_rby1_node',
            name='vive_rby1_node',
            output='screen',
            parameters=[{
                'urdf_path':          urdf_path,
                'srdf_path':          srdf_path,
                'publish_rate':       20.0,
                'ik_dt':              0.05,
                'pos_scale':          0.5,
                'sdk_max_delta_pos':  0.03,
            }],
        ),

        # ── GUI ────────────────────────────────────────────────────────────
        Node(
            package='scm_gui',
            executable='scm_gui_node',
            name='scm_gui',
            output='screen',
        ),

    ])
