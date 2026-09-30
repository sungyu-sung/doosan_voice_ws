#!/usr/bin/env python3
"""
gripper_node.py — ③ 그리퍼 제어 노드  [Module-5]

로봇 팔은 움직이지 않고, 손(그리퍼)만 열었다 닫았다 한다.
집게가 물건을 제대로 잡았는지도 확인한다.

    ros2 launch voice_robot_control gripper.launch.py

MoveIt 이 필요 없으므로 로봇 팔을 안 켜도 실행됩니다.
(다만 실제 그리퍼를 쓰려면 그리퍼가 연결돼 있어야 합니다)

■ 실습
  · OPEN_WIDTH / CLOSE_WIDTH 를 바꿔 집게가 벌어지는 정도를 조절해 보세요.
  · FORCE 를 바꿔 쥐는 힘을 조절해 보세요.
  · 물건을 놓고 / 치우고 실행해 보며 '잡았는지' 판정이 어떻게 달라지는지 보세요.
"""

import rclpy
from rclpy.logging import get_logger

from ..core.gripper_control import connect_gripper, is_gripping, wait_until_done


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 실제 그리퍼를 쓸까?
#   True  = 진짜 움직입니다 (연결이 안 되면 알려주고 시늉만 합니다)
#   False = 그리퍼 없이 연습만 할 때
USE_GRIPPER = True

# 그리퍼 폭 — 단위가 1/10 mm 입니다.  500 = 50mm
OPEN_WIDTH = 500     # 열었을 때 (50mm)
CLOSE_WIDTH = 150    # 닫았을 때 (15mm) — 기어를 쥐는 폭

# 쥐는 힘 — 단위가 1/10 N 입니다.  300 = 30N
FORCE = 300

# 못 잡았을 때 몇 번까지 다시 해볼까?
MAX_RETRY = 3

# ══════════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("gripper_node")
    logger.info("=== ③ 그리퍼 제어 시작 ===")

    gripper, is_real = connect_gripper(logger, USE_GRIPPER)

    # ── 1) 먼저 열기 ────────────────────────────────────
    logger.info(f"그리퍼 열기 ({OPEN_WIDTH / 10:.0f}mm)")
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    # ── 2) 잡을 때까지 닫기 (안 잡히면 다시 시도) ───────
    grip_detected = False

    for attempt in range(1, MAX_RETRY + 1):
        logger.info(
            f"그리퍼 닫기 {attempt}/{MAX_RETRY} 번째 시도 "
            f"(폭={CLOSE_WIDTH / 10:.0f}mm, 힘={FORCE / 10:.0f}N)"
        )
        gripper.move_gripper(width_val=CLOSE_WIDTH, force_val=FORCE)

        # 다 닫힐 때까지 기다린다.
        # 아직 닫히는 중에 확인하면 잡았는데도 '못 잡았다' 가 나온다.
        wait_until_done(gripper, logger)

        grip_detected = is_gripping(gripper, logger)
        logger.info(f"그리퍼 상태: 잡음={grip_detected}")

        if grip_detected:
            logger.info("물건을 제대로 잡았습니다.")
            break

        logger.warning("물건을 못 잡은 것 같습니다. 다시 열고 시도합니다.")
        gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
        wait_until_done(gripper, logger)

    if not grip_detected:
        logger.error(
            f"{MAX_RETRY}번 시도했지만 물건을 잡지 못했습니다.\n"
            "  · 집게 사이에 물건이 있는지 확인하세요\n"
            "  · CLOSE_WIDTH 가 물건보다 큰 건 아닌지 확인하세요"
        )

    # ── 3) 마지막에 다시 열어 두기 ──────────────────────
    logger.info("그리퍼를 다시 열고 마칩니다.")
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    if is_real:
        gripper.close_connection()

    logger.info("=== ③ 그리퍼 제어 끝 ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
