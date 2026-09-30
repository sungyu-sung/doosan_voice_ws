#!/usr/bin/env python3
"""
joint_move_node.py — ①-2 관절 이동 노드  [Module-5]

move_node 는 "손끝을 어디에 둘까(x, y, z)" 를 정해서 움직였다.
이 노드는 반대로 "관절 여섯 개를 각각 몇 도로 돌릴까" 를 정해서 움직인다.

    ros2 launch voice_robot_control joint_move.launch.py

■ 좌표 이동과 무엇이 다른가?
  · 좌표 이동 : 손끝 위치는 내가 정하고, 관절 각도는 로봇이 알아서 푼다.
                → 팔이 어떤 모양으로 접힐지는 그때그때 다르다.
  · 관절 이동 : 관절 각도를 내가 직접 정한다.
                → 팔 모양이 항상 똑같다. 특이점(singularity)도 피할 수 있고,
                  손이 닿기만 하면 실패하지 않는다.

■ 실습: 아래 TARGET 의 숫자를 바꾸고 다시 실행해 보세요.
  단위는 도(°) 입니다.  여섯 개를 다 안 적어도 되고,
  빠진 관절은 홈 값(0, 0, 90, 0, 90, 0)이 됩니다.

  지금 관절이 몇 도인지 보려면:
      ros2 topic echo /joint_states
  (라디안으로 나옵니다. 도로 바꾸려면 ×57.3)

  파일을 고친 뒤 다시 빌드할 필요는 없습니다. 그냥 다시 실행하면 됩니다.
"""

import rclpy
from rclpy.logging import get_logger

from ..core.robot_common import (
    HOME_JOINTS_DEG, finish, go_home, make_home_state, make_joint_state,
    make_plan_params, move_to_joints, setup_robot,
)


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 어떤 자세로 갈까?  (관절 각도, 단위 도°)
#
#   범위   joint_1 : -180 ~ 180      joint_4 : -180 ~ 180
#          joint_2 :  -74 ~  74      joint_5 : -114 ~ 114
#          joint_3 : -114 ~ 114      joint_6 : -180 ~ 180
#   좌표(x, y, z)와 달리 자동으로 잘리지 않습니다.
#   벗어나면 경고가 뜨고 그 자리에서 "계획 실패" 가 됩니다.
#   (범위는 robot_common.py 의 JOINT_RANGE_DEG 에 모아 두었습니다)
TARGET = {
    "joint_1": 30.0,
    "joint_2": 20.0,
    "joint_3": 70.0,
    "joint_4": 0.0,
    "joint_5": 90.0,
    "joint_6": 0.0,
}

# 로봇 속도 (0에 가까울수록 느림. 처음에는 0.1 정도로 천천히)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


def describe(joints_deg) -> str:
    """관절 각도를 로그에 한 줄로 보기 좋게 적는다."""
    parts = []
    for name in HOME_JOINTS_DEG:
        deg = joints_deg.get(name, HOME_JOINTS_DEG[name])
        parts.append(f"{name.replace('joint_', 'J')}={deg:.1f}°")
    return "  ".join(parts)


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("joint_move_node")
    logger.info("=== ①-2 관절 이동 시작 ===")

    robot, arm = setup_robot(logger, node_name="joint_move_node_moveit")

    # 관절 목표는 OMPL(RRTConnect)로 푼다.
    # 좌표가 아니라 관절 각도가 목표라서 Pilz PTP 대신 이쪽을 쓴다.
    joint_params, _ = make_plan_params(robot, vel_home=SPEED, vel_move=SPEED)
    home_state = make_home_state(robot)

    # ── 1) 홈 자세로 ────────────────────────────────────
    if not go_home(robot, arm, logger, home_state, joint_params):
        logger.error("홈 자세로 가지 못해 여기서 멈춥니다.")
        finish(logger)

    # ── 2) 정해둔 관절 자세로 ───────────────────────────
    logger.info(f"=== 목표 자세로 이동: {describe(TARGET)} ===")
    goal_state = make_joint_state(robot, TARGET, logger)

    if move_to_joints(robot, arm, logger, goal_state, joint_params):
        logger.info("=== 도착했습니다 ===")
    else:
        logger.error(
            "=== 목표 자세로 가지 못했습니다 ===\n"
            "  · 관절이 돌 수 있는 범위를 넘었을 수 있습니다\n"
            "  · 팔이 자기 몸이나 책상에 부딪히는 자세일 수 있습니다\n"
            "  → 각도를 조금 줄여서 다시 해 보세요."
        )

    logger.info("=== ①-2 관절 이동 끝 ===")
    finish(logger)


if __name__ == "__main__":
    main()
