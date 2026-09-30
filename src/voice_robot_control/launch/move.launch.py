"""
move.launch.py — move_node 실행

    ros2 launch voice_robot_control move.launch.py

좌표를 터미널에서 바로 줄 수도 있습니다.

    ros2 launch voice_robot_control move.launch.py x:=0.45 y:=0.10 z:=0.40
    ros2 launch voice_robot_control move.launch.py z:=0.60             # z 만
    ros2 launch voice_robot_control move.launch.py degree:=90          # 손목만 90도
    ros2 launch voice_robot_control move.launch.py x:=0.45 speed:=0.05 # 천천히

안 적은 값은 move_node.py 맨 위의 TARGET / SPEED 를 그대로 씁니다.

※ 로봇(또는 시뮬레이터)이 먼저 떠 있어야 합니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from voice_robot_control.launch_helper import config_file, moveit_config, 실수
# 기본값을 노드 파일에서 그대로 가져온다.
# 이렇게 해야 학생이 move_node.py 의 TARGET 을 고쳤을 때 그 값이 그대로 쓰인다.
from voice_robot_control.legacy.move_node import SPEED, TARGET


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value=str(TARGET["x"]),
                              description="목표 x [m]"),
        DeclareLaunchArgument("y", default_value=str(TARGET["y"]),
                              description="목표 y [m]"),
        DeclareLaunchArgument("z", default_value=str(TARGET["z"]),
                              description="목표 z [m]"),
        DeclareLaunchArgument("degree",
                              default_value=str(TARGET.get("degree", 0.0)),
                              description="손목을 돌리는 각도 [도] (-180 ~ 180)"),
        DeclareLaunchArgument("speed", default_value=str(SPEED),
                              description="로봇 속도 (작을수록 느림)"),

        Node(
            package="voice_robot_control",
            executable="move_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                moveit_config().to_dict(),
                config_file("moveit_py.yaml"),
                {
                    "x": 실수("x"),
                    "y": 실수("y"),
                    "z": 실수("z"),
                    "degree": 실수("degree"),
                    "speed": 실수("speed"),
                },
            ],
        ),
    ])
