### sdsmod.launch.py ###
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """"
    システム起動時に何も指定しなければspeech_inputノードを起動する
    ros2 launch diaros_package sdsmod.launch.py mic:=falseでシステムを起動すれば、speech_inputノードは起動しない(ros2 bag playで再生した音声データを元にシステムを動作させる)
    """
    use_mic_arg = DeclareLaunchArgument(
        'mic',
        default_value='true',
        description='Use real microphone input (true/mute/false)'
    )
    use_mic = LaunchConfiguration('mic')

    use_nlg_arg = DeclareLaunchArgument(
        'nlg',
        default_value='true',
        description='Use NLG node (true/false) - Set to false for distributed execution on separate PC'
    )
    use_nlg = LaunchConfiguration('nlg')

    nodes = []
    # use_micが'true'のときのみspeech_inputノードを起動
    nodes.append(
        Node(
            package='diaros_package',
            executable='ros2_speech_input',
            output='screen',
            condition=IfCondition(use_mic)
        )
    )

    nodes += [
        Node(
            package='diaros_package',
            executable='ros2_acoustic_analysis',
            name='acoustic_analysis',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_automatic_speech_recognition',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_natural_language_understanding',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_dialog_management',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_natural_language_generation',
            output='screen',
            condition=IfCondition(use_nlg)
        ),
        Node(
            package='diaros_package',
            executable='ros2_speech_synthesis',
            name='ros2_speech_synthesis',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_speech_output',
            name='ros2_speech_output',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_turn_taking',
            output='screen'
        ),
        Node(
            package='diaros_package',
            executable='ros2_back_channel',
            output='screen',
        ),
    ]

    return LaunchDescription([use_mic_arg, use_nlg_arg] + nodes)