#!/usr/bin/env python3
"""
pick_place_node.py — ④ 집어서 옮기기 노드  [Module-5]

물건 하나를 집어서(pick) 다른 자리에 놓는다(place).
① 팔 움직이기 + ③ 그리퍼 를 처음으로 합쳐 보는 단계다.

    ros2 launch voice_robot_control pick_place.launch.py

동작 순서:

    홈 자세
      ↓
    집는 자리 위       ← 옆에서 부딪히지 않게 위에서 접근한다
      ↓
    집는 자리          그리퍼 닫기 → 잡았는지 확인
      ↓
    다시 위로          들어 올린다
      ↓
    놓는 자리 위       옮긴다
      ↓
    놓는 자리          그리퍼 열기
      ↓
    다시 위로          빠져나온다
      ↓
    홈 자세

■ 실습
  · PICK / PLACE 좌표를 바꿔 다른 자리에서 집고 놓아 보세요.
  · degree 를 바꿔 집게를 비스듬히 돌려서 집어 보세요.
  · APPROACH_OFFSET 을 바꿔 얼마나 높은 데서 접근할지 조절해 보세요.
  · 물건을 치우고 실행하면 '못 잡았다' 는 경고가 뜹니다.
"""

import rclpy
from rclpy.logging import get_logger

from .gripper_control import connect_gripper, is_gripping, wait_until_done
from .robot_common import (
    finish, go_home, make_home_state, make_plan_params, make_pose,
    plan_and_execute, setup_robot,
)


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 어디서 집을까?  (base_link 기준, 단위 m)
#
#   권장 범위   x      :  0.30 ~ 0.60    앞으로 나간 거리
#               y      : -0.30 ~ 0.30    왼쪽(+) / 오른쪽(-)
#               z      :  0.27 ~ 0.60    높이
#               degree : -180 ~ 180      집게를 돌리는 각도 [도]
#   x < 0, |y| > 0.3, z < 0.27 은 자동으로 잘리고 경고가 뜹니다.
#   degree 는 물건이 비스듬히 놓여 있을 때 씁니다. 안 적으면 0 입니다.
#   너무 멀면(팔 길이 0.9m) 범위 안이어도 "계획 실패" 가 뜹니다.
PICK = {"x": 0.427, "y": 0.148, "z": 0.280, "degree": 0}

# 어디에 놓을까?
PLACE = {"x": 0.426, "y": -0.153, "z": 0.280, "degree": 0}

# 집기/놓기 전에 위에서 접근하는 높이 [m]
APPROACH_OFFSET = 0.05

# 그리퍼
#   True  = 진짜 움직입니다 (연결이 안 되면 알려주고 시늉만 합니다)
#   False = 그리퍼 없이 팔 동작만 연습할 때
USE_GRIPPER = True
OPEN_WIDTH = 500        # 열었을 때 (50mm)   ※ 단위가 1/10 mm 입니다
CLOSE_WIDTH = 150       # 닫았을 때 (15mm)
FORCE = 300             # 쥐는 힘 (30N)     ※ 단위가 1/10 N 입니다

# 다 끝내고 홈으로 돌아갈까?
RETURN_HOME = True

# 로봇 속도 (0에 가까울수록 느림)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("pick_place_node")
    logger.info("=== ④ 집어서 옮기기 시작 ===")

    # ── 그리퍼 준비 (먼저 열어 둔다) ────────────────────
    gripper, is_real = connect_gripper(logger, USE_GRIPPER)
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    # ── 로봇 팔 준비 ────────────────────────────────────
    robot, arm = setup_robot(logger, node_name="pick_place_node_moveit")
    home_params, pilz_params = make_plan_params(robot, vel_move=SPEED)
    home_state = make_home_state(robot)

    if not go_home(robot, arm, logger, home_state, home_params):
        logger.error("홈 자세로 가지 못해 여기서 멈춥니다.")
        finish(logger)

    # 중간에 실패하면 여기로 빠져나온다
    def stop_here(what):
        logger.error(f"'{what}' 에서 멈췄습니다. 여기서 끝냅니다.")
        finish(logger)

    # ══════════════════════════════════════════════════
    #   집기 (PICK)
    # ══════════════════════════════════════════════════
    logger.info("=== 집으러 갑니다 ===")

    logger.info("  · 집는 자리 위로")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PICK, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return stop_here("집는 자리 위로")

    logger.info("  · 집는 자리로 내려가기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PICK),
                            plan_parameters=pilz_params):
        return stop_here("집는 자리로 내려가기")

    logger.info(f"  · 그리퍼 닫기 ({CLOSE_WIDTH / 10:.0f}mm)")
    gripper.move_gripper(width_val=CLOSE_WIDTH, force_val=FORCE)
    # 다 닫힐 때까지 기다린다. 안 기다리면 아직 닫히는 중에 팔이 올라가
    # 물건을 놓치거나 다시 잡으려 든다.
    wait_until_done(gripper, logger)

    # ③ 그리퍼 노드에서 배운 '잡았는지 확인' 을 여기서 써먹는다
    if is_gripping(gripper, logger):
        logger.info("  · 물건을 제대로 잡았습니다.")
    else:
        logger.warning(
            "  · 물건을 못 잡은 것 같습니다.\n"
            "      · 집는 자리 좌표(PICK)가 맞는지 확인하세요\n"
            "      · CLOSE_WIDTH 가 물건보다 큰 건 아닌지 확인하세요\n"
            "    → 일단 계속 진행합니다."
        )

    logger.info("  · 들어 올리기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PICK, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return stop_here("들어 올리기")

    # ══════════════════════════════════════════════════
    #   놓기 (PLACE)
    # ══════════════════════════════════════════════════
    logger.info("=== 놓으러 갑니다 ===")

    logger.info("  · 놓는 자리 위로")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PLACE, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return stop_here("놓는 자리 위로")

    logger.info("  · 놓는 자리로 내려가기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PLACE),
                            plan_parameters=pilz_params):
        return stop_here("놓는 자리로 내려가기")

    logger.info(f"  · 그리퍼 열기 ({OPEN_WIDTH / 10:.0f}mm)")
    gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
    wait_until_done(gripper, logger)

    logger.info("  · 빠져나오기")
    if not plan_and_execute(robot, arm, logger,
                            pose_goal=make_pose(PLACE, z_offset=APPROACH_OFFSET),
                            plan_parameters=pilz_params):
        return stop_here("빠져나오기")

    # ── 홈으로 돌아가기 ─────────────────────────────────
    if RETURN_HOME:
        go_home(robot, arm, logger, home_state, home_params)

    if is_real:
        gripper.close_connection()

    logger.info("=== ④ 집어서 옮기기 끝 ===")
    finish(logger)


if __name__ == "__main__":
    main()
