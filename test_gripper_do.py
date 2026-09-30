#!/usr/bin/env python3
"""
Coact(SCHUNK) EGP-C 40 그리퍼 단독 테스트 스크립트.

플랜지(tool) 디지털 출력 1,2번 채널로 그리퍼를 직접 열고/닫아봅니다.
voice_robot_control 패키지 코드는 전혀 건드리지 않는 독립 스크립트입니다.
DO1/DO2 극성이 실제로 "01=열기, 10=닫기"가 맞는지 여기서 먼저 확인하세요.

사용법 (dsr_bringup2가 이미 떠 있는 상태에서, 새 터미널에서):
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 test_gripper_do.py status   # 현재 DO1,DO2 상태 확인
    python3 test_gripper_do.py open     # 열기 (DO1=OFF, DO2=ON)
    python3 test_gripper_do.py close    # 닫기 (DO1=ON,  DO2=OFF)
    python3 test_gripper_do.py raw 1 on # 채널 하나만 수동으로 켜보고 싶을 때
"""

import sys

import rclpy

import DR_init

ROBOT_ID = ""       # 현재 떠 있는 브링업의 name 인자가 비어 있어서 namespace 없음
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

DO_OPEN_ON = 2   # 열기: 이 채널을 ON
DO_OPEN_OFF = 1  # 열기: 이 채널을 OFF
DO_CLOSE_ON = 1  # 닫기: 이 채널을 ON
DO_CLOSE_OFF = 2  # 닫기: 이 채널을 OFF


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("status", "open", "close", "raw"):
        print(__doc__)
        return

    rclpy.init(args=None)
    node = rclpy.create_node("gripper_do_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool_digital_output,
            get_tool_digital_output,
            get_tool_digital_input,
            ON,
            OFF,
        )
    except ImportError as e:
        print(f"DSR_ROBOT2 임포트 실패: {e}")
        print("dsr_bringup2 가 떠 있는지, ros2_ws install/setup.bash 를 source 했는지 확인하세요.")
        rclpy.shutdown()
        return

    import DSR_ROBOT2 as dsr

    for client in (
        dsr._ros2_set_tool_digital_output,
        dsr._ros2_get_tool_digital_output,
        dsr._ros2_get_tool_digital_input,
    ):
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"서비스 {client.srv_name} 가 5초 안에 안 잡힙니다. 브링업이 떠 있는지 확인하세요.")
            rclpy.shutdown()
            return

    def show_status():
        do1 = get_tool_digital_output(1)
        do2 = get_tool_digital_output(2)
        di1 = get_tool_digital_input(1)
        di2 = get_tool_digital_input(2)
        print(f"[출력] DO1={do1}  DO2={do2}   [입력] DI1={di1}  DI2={di2}")

    cmd = sys.argv[1]

    if cmd == "status":
        show_status()

    elif cmd == "open":
        print("여는 중... (DO%d OFF -> DO%d ON)" % (DO_OPEN_OFF, DO_OPEN_ON))
        set_tool_digital_output(DO_OPEN_OFF, OFF)
        set_tool_digital_output(DO_OPEN_ON, ON)
        show_status()

    elif cmd == "close":
        print("닫는 중... (DO%d OFF -> DO%d ON)" % (DO_CLOSE_OFF, DO_CLOSE_ON))
        set_tool_digital_output(DO_CLOSE_OFF, OFF)
        set_tool_digital_output(DO_CLOSE_ON, ON)
        show_status()

    elif cmd == "raw":
        # python3 test_gripper_do.py raw <채널1~6> <on|off>
        ch = int(sys.argv[2])
        val = ON if sys.argv[3].lower() == "on" else OFF
        print(f"채널 {ch} -> {sys.argv[3].upper()}")
        set_tool_digital_output(ch, val)
        show_status()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
