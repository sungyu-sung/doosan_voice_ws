#!/usr/bin/env python3
"""
gripper_control.py — 그리퍼 연결을 도와주는 부품

gripper_node 와 gear_assembly_node 가 함께 쓴다.

실제 그리퍼(EGP-C 40)는 egp_c40.py 가 담당한다.
여기서는 "그리퍼가 없거나 연결이 안 될 때도 수업이 멈추지 않도록"
시늉만 하는 가짜 그리퍼를 대신 넣어 주는 일을 한다.
"""

import time


class DummyGripper:
    """
    시늉만 하는 그리퍼.

    실제 그리퍼와 똑같은 이름의 기능을 갖고 있어서, 노드 쪽 코드는
    진짜인지 가짜인지 신경 쓰지 않아도 된다.
    """

    def __init__(self, logger):
        self._log = logger

    def move_gripper(self, width_val, force_val=400):
        self._log.info(f"[시늉] 그리퍼 이동 (폭={width_val / 10:.0f}mm)")
        time.sleep(0.5)

    def get_status(self):
        # [busy, grip_detected, ...] — 잡은 것으로 쳐 준다
        return [0, 1, 0, 0, 0, 0, 0]

    def close_connection(self):
        pass


def _busy(gripper, logger):
    """그리퍼가 지금 움직이는 중인지. 못 읽으면 None."""
    try:
        return bool(gripper.get_status()[0])
    except Exception as e:
        logger.warning(f"그리퍼 상태를 읽지 못했습니다: {e}")
        return None


def wait_until_done(gripper, logger, timeout: float = 5.0,
                    poll: float = 0.05, start_wait: float = 0.5) -> bool:
    """
    그리퍼가 다 움직일 때까지 기다린다.

    이게 필요한 이유:
      그리퍼에 '닫아라' 라고 말한 뒤 곧바로 팔을 움직이면, 아직 다 닫히지도
      않았는데 팔이 올라가 버려서 물건을 놓치거나 다시 잡으려 든다.

    두 단계로 기다리는 이유:
      명령을 보낸 **직후에는 아직 busy 가 0** 이다. 아직 움직이기 시작을
      안 했기 때문이다. 그때 바로 확인하면 "벌써 다 했네" 라고 착각한다.
      그래서 먼저 움직이기 시작하는 것을 본 다음, 멈추는 것을 기다린다.
    """
    # ── 1단계: 움직이기 시작할 때까지 ──────────────────
    waited = 0.0
    while waited < start_wait:
        busy = _busy(gripper, logger)
        if busy is None:
            return False
        if busy:
            break            # 움직이기 시작했다
        time.sleep(poll)
        waited += poll

    # ── 2단계: 다 움직일 때까지 ────────────────────────
    waited = 0.0
    while waited < timeout:
        busy = _busy(gripper, logger)
        if busy is None:
            return False
        if not busy:
            return True      # 다 움직였다
        time.sleep(poll)
        waited += poll

    logger.warning(f"그리퍼가 {timeout:.0f}초 안에 멈추지 않았습니다.")
    return False


def is_gripping(gripper, logger) -> bool:
    """지금 물건을 잡고 있는지."""
    try:
        return bool(gripper.get_status()[1])
    except Exception as e:
        logger.warning(f"그리퍼 상태를 읽지 못했습니다: {e}")
        return False


def connect_gripper(logger, use_gripper: bool):
    """
    그리퍼를 연결한다. 실패하면 시늉만 하는 그리퍼를 돌려준다.

    돌려주는 값: (그리퍼, 진짜인지 여부)
    """
    if not use_gripper:
        logger.info("그리퍼: 시늉만 합니다 (USE_GRIPPER = False)")
        return DummyGripper(logger), False

    try:
        # EGP-C 40 은 IP/포트 없이 로봇 플랜지 디지털 출력(DO1, DO2)으로 움직인다.
        from .egp_c40 import EGPC40
        gripper = EGPC40()

        logger.info("그리퍼 연결 완료 (EGP-C 40, 플랜지 DO1/DO2)")
        return gripper, True
    except Exception as e:
        logger.error(
            f"그리퍼에 연결하지 못했습니다: {e}\n"
            "  · dsr_bringup2 가 떠 있는지 확인하세요\n"
            "  · ros2_ws 를 source 했는지 확인하세요 (DSR_ROBOT2 를 찾을 수 있어야 합니다)\n"
            "  → 일단 시늉만 하고 계속 진행합니다."
        )
        return DummyGripper(logger), False
