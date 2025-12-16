### sdsmod.launch.py ###
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """"
    基本的にはすべてのノードを起動する
    false を指定したノードだけ起動しない

    例：
      ros2 launch diaros_package sdsmod.launch.py                  # すべてのノード起動
      ros2 launch diaros_package sdsmod.launch.py mic:=false       # speech_input を除外
      ros2 launch diaros_package sdsmod.launch.py nlg:=false       # NLG を除外（分散実行時）
      ros2 launch diaros_package sdsmod.launch.py mic:=false nlg:=false  # speech_input と NLG を除外
    """

    # ノード制御用オプション（すべてデフォルト true で起動）
    use_mic_arg = DeclareLaunchArgument(
        'mic',
        default_value='true',
        description='Use speech_input node (true/false). Set to false for ros2 bag replay.'
    )
    use_mic = LaunchConfiguration('mic')

    use_aa_arg = DeclareLaunchArgument(
        'aa',
        default_value='true',
        description='Use acoustic_analysis node (true/false)'
    )
    use_acoustic = LaunchConfiguration('aa')

    use_asr_arg = DeclareLaunchArgument(
        'asr',
        default_value='true',
        description='Use automatic_speech_recognition node (true/false)'
    )
    use_asr = LaunchConfiguration('asr')

    use_nlu_arg = DeclareLaunchArgument(
        'nlu',
        default_value='true',
        description='Use natural_language_understanding node (true/false)'
    )
    use_nlu = LaunchConfiguration('nlu')

    use_dm_arg = DeclareLaunchArgument(
        'dm',
        default_value='true',
        description='Use dialog_management node (true/false)'
    )
    use_dm = LaunchConfiguration('dm')

    use_nlg_arg = DeclareLaunchArgument(
        'nlg',
        default_value='true',
        description='Use natural_language_generation node (true/false). Set to false for distributed execution on separate PC.'
    )
    use_nlg = LaunchConfiguration('nlg')

    use_ss_arg = DeclareLaunchArgument(
        'ss',
        default_value='true',
        description='Use speech_synthesis node (true/false)'
    )
    use_tts = LaunchConfiguration('ss')

    use_tt_arg = DeclareLaunchArgument(
        'tt',
        default_value='true',
        description='Use turn_taking node (true/false)'
    )
    use_tt = LaunchConfiguration('tt')

    use_bc_arg = DeclareLaunchArgument(
        'bc',
        default_value='true',
        description='Use back_channel node (true/false)'
    )
    use_bc = LaunchConfiguration('bc')

    nodes = [
        Node(
            package='diaros_package',
            executable='ros2_speech_input',
            output='screen',
            condition=IfCondition(use_mic)
        ),
        Node(
            package='diaros_package',
            executable='ros2_acoustic_analysis',
            name='acoustic_analysis',
            output='screen',
            condition=IfCondition(use_acoustic)
        ),
        Node(
            package='diaros_package',
            executable='ros2_automatic_speech_recognition',
            output='screen',
            condition=IfCondition(use_asr)
        ),
        Node(
            package='diaros_package',
            executable='ros2_natural_language_understanding',
            output='screen',
            condition=IfCondition(use_nlu)
        ),
        Node(
            package='diaros_package',
            executable='ros2_dialog_management',
            output='screen',
            condition=IfCondition(use_dm)
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
            output='screen',
            condition=IfCondition(use_tts)
        ),
        Node(
            package='diaros_package',
            executable='ros2_turn_taking',
            output='screen',
            condition=IfCondition(use_tt)
        ),
        Node(
            package='diaros_package',
            executable='ros2_back_channel',
            output='screen',
            condition=IfCondition(use_bc)
        ),
    ]

    return LaunchDescription([
        use_mic_arg,
        use_aa_arg,
        use_asr_arg,
        use_nlu_arg,
        use_dm_arg,
        use_nlg_arg,
        use_ss_arg,
        use_tt_arg,
        use_bc_arg,
    ] + nodes)
