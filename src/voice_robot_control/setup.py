import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'voice_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeptree',
    maintainer_email='deeptree00@gmail.com',
    description='두산 협동로봇 MoveIt2 기본 제어 — pose 기반 동작 노드 + TCP 위치 모니터',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ① 단순 이동  ② 여러 지점  ③ 그리퍼  ④ 집어서 옮기기  ⑤ 기어 조립
            'move_node          = voice_robot_control.legacy.move_node:main',
            'joint_move_node    = voice_robot_control.legacy.joint_move_node:main',
            'waypoint_node      = voice_robot_control.legacy.waypoint_node:main',
            'gripper_node       = voice_robot_control.legacy.gripper_node:main',
            'pick_place_node    = voice_robot_control.legacy.pick_place_node:main',
            'gear_assembly_node = voice_robot_control.legacy.gear_assembly_node:main',
            # ⑥ 음성 인식 (로봇 없이 실행 가능)
            'stt_node           = voice_robot_control.stt_node:main',
            # ⑦ 말 → 로봇 명령 (로봇 없이 실행 가능)
            'nlp_node           = voice_robot_control.nlp_node:main',
            # ⑧ 명령을 받아 실제로 로봇을 움직임
            'voice_robot_node   = voice_robot_control.voice_robot_node:main',
            # ⑨ 팀 프로젝트 — 말로 주문받아 가져다 주기
            'order_node         = voice_robot_control.order_node:main',
            # 손끝 위치 보기 (로봇을 움직이지 않음)
            'position_viewer    = voice_robot_control.position_viewer:main',
        ],
    },
)
