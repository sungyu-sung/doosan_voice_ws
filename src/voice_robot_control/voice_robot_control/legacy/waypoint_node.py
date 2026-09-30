#!/usr/bin/env python3
"""
waypoint_node.py — ② 여러 지점 지나가기 노드  [Module-5]

홈 자세로 간 다음, 정해둔 지점들을 순서대로 지나간다.
① 단순 이동을 여러 번 이어 붙인 것이라고 보면 된다.

    ros2 launch voice_robot_control waypoint.launch.py

■ 실습
  · WAYPOINTS 에 줄을 더 넣으면 지나가는 지점이 늘어납니다.
  · 순서를 바꾸면 지나가는 순서가 바뀝니다.
  · REPEAT 를 2로 바꾸면 같은 길을 두 번 돕니다.
  · degree 를 넣으면 지나가면서 손목을 돌립니다.
"""

import rclpy
from rclpy.logging import get_logger

from ..core.robot_common import (
    describe_pos, finish, go_home, make_home_state, make_plan_params,
    make_pose, plan_and_execute, setup_robot,
)


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 지나갈 지점들 (base_link 기준, 단위 m)
# 위에서부터 순서대로 지나갑니다.
#
#   권장 범위   x      :  0.30 ~ 0.60    앞으로 나간 거리
#               y      : -0.30 ~ 0.30    왼쪽(+) / 오른쪽(-)
#               z      :  0.27 ~ 0.60    높이
#               degree : -180 ~ 180      손목을 돌리는 각도 [도]
#   x < 0, |y| > 0.3, z < 0.27 은 자동으로 잘리고 경고가 뜹니다.
#   degree 는 안 적으면 0 (지금까지와 같음) 으로 봅니다.
#   너무 멀면(팔 길이 0.9m) 범위 안이어도 "계획 실패" 가 뜹니다.
WAYPOINTS = [
    {"x": 0.493, "y":  0.010, "z": 0.417},
    {"x": 0.493, "y": -0.218, "z": 0.417, "degree": 45},
    {"x": 0.371, "y": -0.218, "z": 0.419, "degree": 90},
    {"x": 0.371, "y":  0.010, "z": 0.419},
]

# 몇 바퀴 돌까?
REPEAT = 1

# 다 돌고 나서 홈으로 돌아갈까?
RETURN_HOME = True

# 로봇 속도 (0에 가까울수록 느림)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("waypoint_node")
    logger.info("=== ② 여러 지점 지나가기 시작 ===")

    robot, arm = setup_robot(logger, node_name="waypoint_node_moveit")
    home_params, pilz_params = make_plan_params(robot, vel_move=SPEED)
    home_state = make_home_state(robot)

    # ── 홈 자세로 ───────────────────────────────────────
    if not go_home(robot, arm, logger, home_state, home_params):
        logger.error("홈 자세로 가지 못해 여기서 멈춥니다.")
        finish(logger)

    total = len(WAYPOINTS)
    logger.info(f"=== 지점 {total}개를 {REPEAT}바퀴 돕니다 ===")

    # ── 지점들을 순서대로 ───────────────────────────────
    for lap in range(1, REPEAT + 1):
        if REPEAT > 1:
            logger.info(f"===== {lap}/{REPEAT} 바퀴 =====")

        for i, point in enumerate(WAYPOINTS, start=1):
            logger.info(
                f"--- {i}/{total} 번째 지점: {describe_pos(point)} ---")

            pose_goal = make_pose(point)
            if not plan_and_execute(robot, arm, logger,
                                    pose_goal=pose_goal,
                                    plan_parameters=pilz_params):
                logger.error(f"{i}/{total} 번째 지점에서 멈췄습니다. 여기서 끝냅니다.")
                finish(logger)

            logger.info(f"--- {i}/{total} 번째 지점 도착 ---")

    # ── 홈으로 돌아가기 ─────────────────────────────────
    if RETURN_HOME:
        go_home(robot, arm, logger, home_state, home_params)

    logger.info("=== ② 여러 지점 지나가기 끝 ===")
    finish(logger)


if __name__ == "__main__":
    main()
