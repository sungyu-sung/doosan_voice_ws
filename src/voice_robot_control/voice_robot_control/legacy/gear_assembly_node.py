#!/usr/bin/env python3
"""
gear_assembly_node.py — ⑤ 기어 조립 노드  [Module-5]

기어를 집어서(pick) 옮겨 놓는(place) 작업을 순서대로 한다.
④ 집어서 옮기기 를 기어 여러 개에 대해 반복하는 것이다.

    ros2 launch voice_robot_control gear.launch.py

기어 하나를 조립하는 순서:

    집는 자리 위       ← 옆에서 부딪히지 않게 위에서 접근한다
      ↓
    집는 자리          그리퍼 닫기
      ↓
    다시 위로          들어 올린다
      ↓
    놓는 자리 위       옮긴다
      ↓
    놓는 자리          그리퍼 열기
      ↓
    다시 위로          빠져나온다

■ 실습
  · GEAR_TASKS 의 좌표를 바꿔 기어 위치를 맞춰 보세요.
  · degree 를 넣으면 집게를 그만큼 돌려서 집고 놓습니다.
  · 순서를 바꾸면 조립 순서가 바뀝니다.
  · USE_WIGGLE 을 False 로 하면 마지막에 좌우로 흔드는 동작을 건너뜁니다.
"""

import rclpy
from rclpy.logging import get_logger

from ..core.gripper_control import connect_gripper, is_gripping, wait_until_done
from ..core.robot_common import (
    describe_pos, finish, go_home, make_home_state, make_plan_params,
    make_pose, plan_and_execute, setup_robot,
)


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 기어를 어디서 집어 어디에 놓을지 (base_link 기준, 단위 m)
# 위에서부터 순서대로 조립합니다.
#
#   권장 범위   x      :  0.30 ~ 0.60    앞으로 나간 거리
#               y      : -0.30 ~ 0.30    왼쪽(+) / 오른쪽(-)
#               z      :  0.27 ~ 0.60    높이
#               degree : -180 ~ 180      집게를 돌리는 각도 [도]
#   x < 0, |y| > 0.3, z < 0.27 은 자동으로 잘리고 경고가 뜹니다.
#   degree 는 기어가 비스듬히 놓여 있을 때 씁니다. 안 적으면 0 입니다.
#   너무 멀면(팔 길이 0.9m) 범위 안이어도 "계획 실패" 가 뜹니다.
GEAR_TASKS = [
    {   # 1번 기어
        "pick":  {"x": 0.393, "y":  0.094, "z": 0.280, "degree": 0},
        "place": {"x": 0.393, "y": -0.206, "z": 0.280, "degree": 0},
    },
    {   # 2번 기어
        "pick":  {"x": 0.392, "y":  0.200, "z": 0.280, "degree": 0},
        "place": {"x": 0.392, "y": -0.101, "z": 0.280, "degree": 0},
    },
    {   # 3번 기어
        "pick":  {"x": 0.486, "y":  0.153, "z": 0.280, "degree": 0},
        "place": {"x": 0.486, "y": -0.149, "z": 0.280, "degree": 0},
    },
    {   # 4번 기어
        "pick":  {"x": 0.427, "y":  0.148, "z": 0.280, "degree": 0},
        "place": {"x": 0.426, "y": -0.153, "z": 0.280, "degree": 0},
    },
]

# 집기/놓기 전에 위에서 접근하는 높이 [m]
APPROACH_OFFSET = 0.05

# 마지막 기어를 끼워 넣을 때 좌우로 살짝 흔들기
USE_WIGGLE = True
WIGGLE_Z = 0.295        # 흔드는 높이 [m]
WIGGLE_YAW_DEG = 5.0    # 좌우로 도는 각도 [도]
WIGGLE_COUNT = 3        # 좌우 왕복 횟수

# 그리퍼
#   True  = 진짜 움직입니다 (연결이 안 되면 알려주고 시늉만 합니다)
#   False = 그리퍼 없이 팔 동작만 연습할 때
USE_GRIPPER = True
OPEN_WIDTH = 500        # 열었을 때 (50mm)
CLOSE_WIDTH = 150       # 닫았을 때 (15mm)
FORCE = 300             # 쥐는 힘 (30N)

# 로봇 속도 (0에 가까울수록 느림)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


def do_wiggle(robot, arm, logger, place, pilz_params):
    """
    마지막 기어를 끼울 때 좌우로 살짝 흔든다.

    흔드는 것도 결국 '손목을 돌리는 것' 이라, place 의 degree 에서
    ±WIGGLE_YAW_DEG 만큼 더한 자리로 왔다 갔다 하면 된다.
    """
    기준각도 = place.get("degree", 0.0)
    logger.info(
        f"좌우 흔들기: z={WIGGLE_Z:.3f} 에서 "
        f"±{WIGGLE_YAW_DEG}도 씩 {WIGGLE_COUNT}번"
    )

    def 흔들자리(각도):
        return {"x": place["x"], "y": place["y"], "z": WIGGLE_Z,
                "degree": 기준각도 + 각도}

    # 흔들 높이까지 올라간다
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(흔들자리(0.0)),
                            plan_parameters=pilz_params):
        return False

    for i in range(1, WIGGLE_COUNT + 1):
        for sign, mark in ((+1, "+"), (-1, "-")):
            logger.info(f"흔들기 {i}/{WIGGLE_COUNT}: {mark}{WIGGLE_YAW_DEG:.1f}도")
            if not plan_and_execute(
                    robot, arm, logger,
                    pose_goal=make_pose(흔들자리(sign * WIGGLE_YAW_DEG)),
                    plan_parameters=pilz_params):
                return False

    # 원래 자세로 되돌린다
    return plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(흔들자리(0.0)),
                            plan_parameters=pilz_params)


def assemble_one(robot, arm, logger, gripper, task, pilz_params, is_last):
    """기어 하나를 집어서 놓는다. 성공하면 True."""
    pick, place = task["pick"], task["place"]
    logger.info(f"  집는 자리 {describe_pos(pick)}")
    logger.info(f"  놓는 자리 {describe_pos(place)}")

    # 1) 집는 자리 위로
    logger.info("  · 집는 자리 위로")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(pick, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return False

    # 2) 집는 자리로 내려가기
    logger.info("  · 집는 자리로 내려가기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(pick),
                            plan_parameters=pilz_params):
        return False

    # 3) 그리퍼 닫기 (집기)
    logger.info(f"  · 그리퍼 닫기 ({CLOSE_WIDTH / 10:.0f}mm) — 기어 집기")
    gripper.move_gripper(width_val=CLOSE_WIDTH, force_val=FORCE)
    # 다 닫힐 때까지 기다린다. 안 기다리면 아직 닫히는 중에 팔이 올라가
    # 기어를 놓치거나 다시 잡으려 든다.
    wait_until_done(gripper, logger)
    if not is_gripping(gripper, logger):
        logger.warning("  · 기어를 못 잡은 것 같습니다 — 일단 계속 진행합니다.")

    # 4) 다시 위로
    logger.info("  · 들어 올리기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(pick, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return False

    # 5) 놓는 자리 위로
    logger.info("  · 놓는 자리 위로")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(place, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return False

    # 6) 마지막 기어면 좌우로 흔들어 끼운다
    if is_last and USE_WIGGLE:
        if not do_wiggle(robot, arm, logger, place, pilz_params):
            return False

    # 7) 놓는 자리로 내려가기
    logger.info("  · 놓는 자리로 내려가기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(place),
                            plan_parameters=pilz_params):
        return False

    # 8) 그리퍼 열기 (놓기)
    logger.info(f"  · 그리퍼 열기 ({OPEN_WIDTH / 10:.0f}mm) — 기어 놓기")
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    # 9) 다시 위로 빠져나오기
    logger.info("  · 빠져나오기")
    return plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(place, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params)


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("gear_assembly_node")
    logger.info("=== ⑤ 기어 조립 시작 ===")

    # ── 그리퍼 준비 (먼저 열어 둔다) ────────────────────
    gripper, is_real = connect_gripper(logger, USE_GRIPPER)
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    # ── 로봇 팔 준비 ────────────────────────────────────
    robot, arm = setup_robot(logger, node_name="gear_node_moveit")
    home_params, pilz_params = make_plan_params(robot, vel_move=SPEED)
    home_state = make_home_state(robot)

    if not go_home(robot, arm, logger, home_state, home_params):
        logger.error("홈 자세로 가지 못해 여기서 멈춥니다.")
        finish(logger)

    # ── 기어를 하나씩 조립 ──────────────────────────────
    total = len(GEAR_TASKS)
    logger.info(f"=== 기어 {total}개 조립 시작 ===")

    for i, task in enumerate(GEAR_TASKS, start=1):
        logger.info(f"----- {i}/{total} 번째 기어 시작 -----")

        if not assemble_one(robot, arm, logger, gripper, task,
                            pilz_params, is_last=(i == total)):
            logger.error(f"{i}/{total} 번째 기어에서 멈췄습니다. 여기서 끝냅니다.")
            go_home(robot, arm, logger, home_state, home_params)
            finish(logger)

        logger.info(f"----- {i}/{total} 번째 기어 완료 -----")

    # ── 다 끝내고 홈으로 ────────────────────────────────
    logger.info("=== 모든 기어 조립 완료. 홈으로 돌아갑니다 ===")
    go_home(robot, arm, logger, home_state, home_params)

    if is_real:
        gripper.close_connection()

    logger.info("=== ⑤ 기어 조립 끝 ===")
    finish(logger)


if __name__ == "__main__":
    main()
