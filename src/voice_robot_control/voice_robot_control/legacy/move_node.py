#!/usr/bin/env python3
"""
move_node.py — ① 단순 이동 노드  [Module-5]

로봇을 홈 자세로 보낸 다음, 정해진 좌표 한 곳으로 이동한다.
가장 기본이 되는 노드다.

    ros2 launch voice_robot_control move.launch.py

■ 실습 1 — 파일 고치기
  아래 TARGET 의 숫자를 바꾸고 다시 실행해 보세요.
  x, y, z 의 단위는 m 입니다.  (0.5 = 50cm)
  degree 는 손목을 돌리는 각도입니다. 단위는 도(°).
  파일을 고친 뒤 다시 빌드할 필요는 없습니다. 그냥 다시 실행하면 됩니다.

■ 실습 2 — 터미널에서 바로 찍기 (파일을 안 고쳐도 됩니다)

      ros2 launch voice_robot_control move.launch.py x:=0.45 y:=0.10 z:=0.40

  안 적은 값은 아래 TARGET / SPEED 의 값을 그대로 씁니다.

      ros2 launch voice_robot_control move.launch.py z:=0.60            # z 만 바꾸기
      ros2 launch voice_robot_control move.launch.py degree:=90         # 손목만 돌리기
      ros2 launch voice_robot_control move.launch.py x:=0.45 speed:=0.05 # 천천히
"""

import rclpy


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 어디로 갈까?  (base_link 기준, 단위 m)
#
#   권장 범위   x      :  0.30 ~ 0.60    앞으로 나간 거리
#               y      : -0.30 ~ 0.30    왼쪽(+) / 오른쪽(-)
#               z      :  0.27 ~ 0.60    높이
#               degree : -180 ~ 180      손목을 돌리는 각도 [도]
#   x < 0, |y| > 0.3, z < 0.27 은 자동으로 잘리고 경고가 뜹니다.
#   degree 는 0 이면 지금까지와 같고, 90 이면 집게를 90도 돌린 채로 갑니다.
#   너무 멀면(팔 길이 0.9m) 범위 안이어도 "계획 실패" 가 뜹니다.
TARGET = {"x": 0.500, "y": 0.000, "z": 0.500, "degree": 0.0}

# 로봇 속도 (0에 가까울수록 느림. 처음에는 0.1 정도로 천천히)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


def read_target(node, logger):
    """
    목표 좌표를 정한다.

    터미널에서 `x:=0.45` 처럼 주면 그 값을, 안 주면 위의 TARGET 값을 쓴다.
    """
    기본값 = {name: float(TARGET.get(name, 0.0))
             for name in ("x", "y", "z", "degree")}
    기본값["speed"] = float(SPEED)

    for name, value in 기본값.items():
        node.declare_parameter(name, value)

    준값 = {name: node.get_parameter(name).value for name in 기본값}

    바뀐것 = [name for name, value in 준값.items() if value != 기본값[name]]
    if 바뀐것:
        logger.info(
            f"터미널에서 준 값을 씁니다 (파일의 TARGET 대신): "
            f"{', '.join(f'{n}={준값[n]:g}' for n in 바뀐것)}")

    target = {name: 준값[name] for name in ("x", "y", "z", "degree")}
    return target, 준값["speed"]


def main(args=None):
    # 로봇 관련 도구는 여기서 불러온다.
    # (move.launch.py 가 위의 TARGET 값을 읽어갈 때 MoveIt 까지 필요하지 않도록)
    from ..core.robot_common import (
        describe_pos, finish, go_home, make_home_state, make_plan_params,
        make_pose, plan_and_execute, setup_robot,
    )

    rclpy.init(args=args)
    node = rclpy.create_node("move_node")
    logger = node.get_logger()
    logger.info("=== ① 단순 이동 시작 ===")

    target, speed = read_target(node, logger)

    robot, arm = setup_robot(logger, node_name="move_node_moveit")
    home_params, pilz_params = make_plan_params(robot, vel_move=speed)
    home_state = make_home_state(robot)

    # ── 1) 홈 자세로 ────────────────────────────────────
    if not go_home(robot, arm, logger, home_state, home_params):
        logger.error("홈 자세로 가지 못해 여기서 멈춥니다.")
        finish(logger)

    # ── 2) 정해둔 좌표로 ────────────────────────────────
    logger.info(
        f"=== 목표 좌표로 이동: {describe_pos(target)} (속도 {speed}) ===")
    pose_goal = make_pose(target)

    if plan_and_execute(robot, arm, logger,
                        pose_goal=pose_goal,
                        plan_parameters=pilz_params):
        logger.info("=== 도착했습니다 ===")
    else:
        logger.error("=== 목표 좌표로 가지 못했습니다 ===")

    logger.info("=== ① 단순 이동 끝 ===")
    finish(logger)


if __name__ == "__main__":
    main()
