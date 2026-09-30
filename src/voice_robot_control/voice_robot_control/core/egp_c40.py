#!/usr/bin/env python3
"""
egp_c40.py — Coact(SCHUNK) EGP-C 40 그리퍼 드라이버 (DA 그리퍼)

플랜지(tool) 디지털 출력 1, 2번 채널로 열고 닫는다.
    열기  : DO1=OFF, DO2=ON   (01)
    닫기  : DO1=ON,  DO2=OFF  (10)

노드들이 기대하는 그리퍼 기능(move_gripper, get_status, close_connection)을
갖추고 있어서, 노드 쪽 코드는 그리퍼 종류를 신경 쓰지 않아도 된다.

DSR_ROBOT2 의 함수는 안에서 spin 을 또 돌려서, 노드가 spin 중일 때
(명령 콜백 안에서) 부르면 "generator already executing" 이 난다.
그래서 서비스를 직접 부르고, 응답은 별도 스레드의 executor 가 받는다.
"""

import threading
import time

DO_OPEN_ON, DO_OPEN_OFF = 2, 1
DO_CLOSE_ON, DO_CLOSE_OFF = 1, 2

# 폭(1/10 mm) 이 이 값 이상이면 "열기", 미만이면 "닫기". (열림 500, 닫힘 150 의 중간)
OPEN_THRESHOLD = 325

# 상태 피드백이 없어서, 명령 후 이 시간 동안은 "움직이는 중"으로 친다. (초)
MOVE_TIME = 1.5

ON, OFF = 1, 0


class EGPC40:
    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from dsr_msgs2.srv import SetToolDigitalOutput

        self._node = rclpy.create_node("egp_c40_gripper")
        self._req_type = SetToolDigitalOutput.Request
        self._client = self._node.create_client(
            SetToolDigitalOutput, "/io/set_tool_digital_output")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

        if not self._client.wait_for_service(timeout_sec=5.0):
            raise ConnectionError(
                "/io/set_tool_digital_output 서비스가 안 잡힙니다 (dsr_bringup2 확인)")

        self._moved_at = 0.0

    def _set(self, index, value):
        req = self._req_type()
        req.index = index
        req.value = value
        done = threading.Event()
        future = self._client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=3.0):
            raise TimeoutError(f"DO{index} 설정 응답이 없습니다")
        if not future.result().success:
            raise RuntimeError(f"DO{index} 설정에 실패했습니다")

    def move_gripper(self, width_val, force_val=None):
        """폭이 크면 열고 작으면 닫는다. 힘은 지정할 수 없어서 무시한다."""
        if width_val >= OPEN_THRESHOLD:
            self._set(DO_OPEN_OFF, OFF)
            self._set(DO_OPEN_ON, ON)
        else:
            self._set(DO_CLOSE_OFF, OFF)
            self._set(DO_CLOSE_ON, ON)
        self._moved_at = time.monotonic()

    def get_status(self):
        # [busy, grip_detected, ...] — 피드백이 없어서 busy 는 시간으로 추정, 파지는 항상 잡은 것으로 친다
        busy = int(time.monotonic() - self._moved_at < MOVE_TIME)
        return [busy, 1, 0, 0, 0, 0, 0]

    def close_connection(self):
        self._executor.shutdown()
        self._node.destroy_node()
