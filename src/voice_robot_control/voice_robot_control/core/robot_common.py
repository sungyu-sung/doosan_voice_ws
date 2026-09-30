#!/usr/bin/env python3
"""
robot_common.py — 네 개의 노드가 함께 쓰는 부분

move_node / waypoint_node / gear_assembly_node 가 로봇을 움직일 때
똑같이 필요한 것들을 여기 모아 두었다.

  · 로봇 이름표 (그룹 이름, 프레임 이름 ...)
  · 홈 자세
  · 안전 작업 영역
  · 경로 계획 + 실행 함수

수업 중에 바꿀 일이 있다면 아래 '안전 작업 영역' 정도다.
각 노드가 어디로 움직일지는 그 노드 파일 맨 위에 적혀 있다.
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .pose_utils import wrap_deg, wrist_deg_to_quat


# ══════════════════════════════════════════════════════════
#  로봇 이름표 — 두산 M0609
# ══════════════════════════════════════════════════════════
GROUP_NAME = "manipulator"   # SRDF에 정의된 planning group 이름
BASE_FRAME = "base_link"     # 로봇 베이스 프레임
EE_LINK = "link_6"           # 손끝(엔드이펙터) 링크 이름

# 홈 자세 — 관절 6개의 각도 [도]
HOME_JOINTS_DEG = {
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": 90.0,
    "joint_4": 0.0,
    "joint_5": 90.0,
    "joint_6": 0.0,
}

# 관절이 돌 수 있는 범위 [도]
# dsr_moveit_config_m0609/config/joint_limits.yaml 의 값을 도로 바꾼 것이다.
# 관절 각도는 좌표와 달리 자동으로 잘리지 않는다. 벗어나면 "계획 실패" 가 뜬다.
JOINT_RANGE_DEG = {
    "joint_1": (-180.0, 180.0),
    "joint_2": (-74.0, 74.0),
    "joint_3": (-114.0, 114.0),
    "joint_4": (-180.0, 180.0),
    "joint_5": (-114.0, 114.0),
    "joint_6": (-180.0, 180.0),
}

# 손끝이 바닥을 바라보는 자세 (쿼터니언 x, y, z, w)
# = degree 0 인 손목 자세와 같다.
DOWN = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}


# ══════════════════════════════════════════════════════════
#  안전 작업 영역 (base_link 기준)
#  이 범위를 벗어난 목표는 경계값으로 잘라낸다
# ══════════════════════════════════════════════════════════
SAFE_X_MIN = 0.0      # x는 0 이상
SAFE_Y_MIN = -0.3     # y 하한
SAFE_Y_MAX = 0.3      # y 상한
SAFE_Z_MIN = 0.27     # z는 이 값보다 낮아지면 안 됨


# ══════════════════════════════════════════════════════════
#  이동을 마친 뒤 잠깐 기다리는 시간 [초]
#
#  로봇 제어기는 "다 보냈다" 고 알려주지만, 실제 팔은 아직 조금 더
#  움직이고 있다. 그 상태에서 곧바로 그리퍼를 여닫으면 도착하기도 전에
#  집게가 움직여 버린다. 그래서 매 이동 뒤에 잠깐 기다린다.
#
#  로봇이 흔들림 없이 잘 멈춘다면 0.3 정도로 줄여도 된다.
# ══════════════════════════════════════════════════════════
SETTLE_TIME = 0.5


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """안전 작업 영역 안으로 (x, y, z) 를 끌어들인다."""
    safe_x, safe_y, safe_z = x, y, z

    if safe_x < SAFE_X_MIN:
        logger.warning(f"x 가 안전 범위를 벗어나 {safe_x:.3f} → {SAFE_X_MIN:.3f} 로 줄임")
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(f"y 가 안전 범위를 벗어나 {safe_y:.3f} → {SAFE_Y_MIN:.3f} 로 줄임")
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(f"y 가 안전 범위를 벗어나 {safe_y:.3f} → {SAFE_Y_MAX:.3f} 로 줄임")
        safe_y = SAFE_Y_MAX

    if safe_z < SAFE_Z_MIN:
        logger.warning(f"z 가 안전 범위를 벗어나 {safe_z:.3f} → {SAFE_Z_MIN:.3f} 로 줄임")
        safe_z = SAFE_Z_MIN

    return safe_x, safe_y, safe_z


# ══════════════════════════════════════════════════════════
#  경로 계획 + 실행
# ══════════════════════════════════════════════════════════
def plan_and_execute(robot, arm, logger, pose_goal=None, plan_parameters=None):
    """
    갈 길을 계산하고 그대로 움직인다.

    pose_goal 을 주면 안전 영역으로 잘라낸 뒤 그 좌표를 목표로 삼는다.
    pose_goal 을 안 주면 미리 정해둔 목표(예: 관절 자세)를 쓴다.

    성공하면 True, 실패하면 False 를 돌려준다.
    """
    if pose_goal is not None:
        sx, sy, sz = clamp_to_safe_workspace(
            pose_goal.pose.position.x,
            pose_goal.pose.position.y,
            pose_goal.pose.position.z,
            logger,
        )
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz

        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)

    # ── 1단계: 갈 길 계산 (로봇은 아직 안 움직인다) ──
    logger.info("경로 계산 중 ...")
    if plan_parameters is not None:
        plan_result = arm.plan(parameters=plan_parameters)
    else:
        plan_result = arm.plan()

    if not plan_result:
        logger.error(
            "가는 길을 찾지 못했습니다.\n"
            "  · 로봇 팔이 닿지 않는 곳일 수 있습니다\n"
            "  · 다른 물체에 부딪히는 위치일 수 있습니다\n"
            "  → 좌표를 조금 바꿔서 다시 해 보세요."
        )
        return False

    # ── 2단계: 계산한 길대로 실제로 움직인다 ──
    logger.info("이동 중 ...")
    status = robot.execute(
        group_name=GROUP_NAME,
        robot_trajectory=plan_result.trajectory,
        blocking=True,
    )

    # 결과를 꼭 확인한다. 확인하지 않으면 로봇이 안 움직였는데도
    # '완료' 라고 알려주게 된다.
    if status is not None and not bool(status):
        reason = getattr(status, "status", "알 수 없음")
        logger.error(
            f"길은 찾았는데 로봇이 움직이지 않았습니다 (사유: {reason})\n"
            "  · 로봇 드라이버가 꺼져 있거나 연결이 끊겼을 수 있습니다\n"
            "  · 로봇이 비상정지 상태일 수 있습니다\n"
            "  → 로봇을 켠 터미널의 화면을 확인하세요."
        )
        return False

    # 팔이 완전히 멈출 때까지 잠깐 기다린다.
    # 이걸 빼면 아직 움직이는 중에 그리퍼가 여닫힌다.
    time.sleep(SETTLE_TIME)

    logger.info("이동 완료")
    return True


# ══════════════════════════════════════════════════════════
#  준비물 만들기
# ══════════════════════════════════════════════════════════
def setup_robot(logger, node_name="moveit_py"):
    """MoveIt 을 켜고 팔을 쓸 준비를 한다."""
    logger.info("MoveIt 준비 중 ... (10초쯤 걸립니다)")
    robot = MoveItPy(node_name=node_name)
    arm = robot.get_planning_component(GROUP_NAME)
    logger.info("MoveIt 준비 완료")
    return robot, arm


def make_plan_params(robot, vel_home=0.2, vel_move=0.15, acc=0.1, plan_time=2.0):
    """
    두 가지 계획 방법을 만든다.

      home_params : 관절을 크게 돌릴 때 (OMPL)
      pilz_params : 좌표로 곧장 갈 때 (Pilz PTP)
    """
    home_params = PlanRequestParameters(robot)
    home_params.planning_pipeline = "ompl"
    # 이름은 ompl_planning.yaml 의 planner_configs 에 있는 것과 똑같아야 한다.
    # 틀리면 "Cannot find planning configuration" 경고가 뜨고 기본값으로 돈다.
    home_params.planner_id = "RRTConnect"
    home_params.max_velocity_scaling_factor = vel_home
    home_params.max_acceleration_scaling_factor = acc
    home_params.planning_time = plan_time

    pilz_params = PlanRequestParameters(robot)
    pilz_params.planning_pipeline = "pilz_industrial_motion_planner"
    pilz_params.planner_id = "PTP"
    pilz_params.max_velocity_scaling_factor = vel_move
    pilz_params.max_acceleration_scaling_factor = acc
    pilz_params.planning_time = plan_time

    return home_params, pilz_params


def make_joint_state(robot, joints_deg, logger=None):
    """
    관절 각도[도] 로 RobotState 를 만든다.

      joints_deg : {"joint_1": 0.0, "joint_2": ...}  — 단위는 도(°)

    여섯 개를 다 적지 않아도 된다. 빠진 관절은 홈 자세 값으로 채운다.
    (RobotState 는 안 적은 관절을 0도로 두기 때문에, 그대로 두면
     생각지도 못한 자세로 팔이 움직인다.)
    """
    angles = dict(HOME_JOINTS_DEG)

    for name, deg in joints_deg.items():
        if name not in angles:
            if logger is not None:
                logger.warning(f"'{name}' 은(는) 없는 관절 이름입니다. 무시합니다.")
            continue
        angles[name] = float(deg)

        # 범위를 벗어나면 계획 단계에서 실패한다. 왜 실패했는지 미리 알려준다.
        low, high = JOINT_RANGE_DEG[name]
        if not low <= float(deg) <= high and logger is not None:
            logger.warning(
                f"{name} = {float(deg):g}도 는 돌 수 있는 범위"
                f"({low:g} ~ {high:g}도) 를 벗어났습니다. 계획이 실패할 수 있습니다.")

    missing = [n for n in HOME_JOINTS_DEG if n not in joints_deg]
    if missing and logger is not None:
        logger.info(f"적지 않은 관절은 홈 값으로 둡니다: {', '.join(missing)}")

    state = RobotState(robot.get_robot_model())
    state.joint_positions = {
        name: math.radians(deg) for name, deg in angles.items()
    }
    state.update()
    return state


def make_home_state(robot):
    """홈 자세를 나타내는 RobotState 를 만든다."""
    return make_joint_state(robot, HOME_JOINTS_DEG)


def move_to_joints(robot, arm, logger, joint_state, plan_parameters):
    """관절 자세(RobotState) 하나로 이동한다."""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=joint_state)
    return plan_and_execute(robot, arm, logger, plan_parameters=plan_parameters)


def go_home(robot, arm, logger, home_state, home_params):
    """홈 자세로 이동."""
    logger.info("=== 홈 자세로 이동 ===")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=home_state)
    return plan_and_execute(robot, arm, logger, plan_parameters=home_params)


def current_tcp(robot, logger):
    """
    지금 손끝이 어디에 어떤 방향으로 있는지 알아온다.

    돌려주는 값: (위치, 방향) — 둘 다 make_pose() 에 바로 넣을 수 있는 모양.
    못 읽으면 (None, None).
    """
    try:
        with robot.get_planning_scene_monitor().read_only() as scene:
            state = scene.current_state
            state.update()
            tf = state.get_frame_transform(EE_LINK)
    except Exception as e:
        logger.error(f"지금 위치를 알 수 없습니다: {e}")
        return None, None

    pos = {"x": float(tf[0, 3]), "y": float(tf[1, 3]), "z": float(tf[2, 3])}

    r = tf[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w, x = 0.25 / s, (r[2, 1] - r[1, 2]) * s
        y, z = (r[0, 2] - r[2, 0]) * s, (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x = (r[2, 1] - r[1, 2]) / s, 0.25 * s
        y, z = (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s
        y, z = 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s
        y, z = (r[1, 2] + r[2, 1]) / s, 0.25 * s

    return pos, {"x": float(x), "y": float(y), "z": float(z), "w": float(w)}


def finish(logger):
    """
    노드를 깔끔하게 끝낸다.

    MoveIt 은 파이썬이 아니라 C++ 로 돌아가는 부분이 많은데, 파이썬이 먼저
    정리를 시작하면 그 뒤처리 도중에 프로그램이 비정상 종료(-11)한다.
    할 일은 이미 다 끝난 뒤라 로봇에는 문제가 없지만, 화면에 빨간 글씨로
    "process has died" 가 떠서 학생이 실패한 줄 알게 된다.
    그래서 정리를 기다리지 않고 여기서 바로 끝낸다.
    """
    import os
    import sys

    logger.info("프로그램을 마칩니다.")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def wrist_ori(degree=0.0) -> dict:
    """손목만 degree[도] 돌린 자세를 make_pose 에 넣을 모양으로 만든다."""
    x, y, z, w = wrist_deg_to_quat(degree)
    return {"x": x, "y": y, "z": z, "w": w}


def describe_pos(pos, z_offset=0.0) -> str:
    """좌표를 로그에 한 줄로 적는다. degree 를 적어 두었으면 그것도 같이."""
    text = (f"x={float(pos['x']):.3f}, y={float(pos['y']):.3f}, "
            f"z={float(pos['z']) + z_offset:.3f}")
    degree = pos.get("degree", 0.0)
    if degree:
        text += f", 손목={wrap_deg(degree):g}도"
    return text


def make_pose(pos, ori=None, z_offset=0.0) -> PoseStamped:
    """
    좌표와 자세로 PoseStamped 를 만든다.

      pos      : {"x":..., "y":..., "z":...}
                 "degree" 를 같이 적어 두면 손목을 그만큼 돌린 채로 간다.
      ori      : {"x":..., "y":..., "z":..., "w":...}
                 생략하면 pos 의 "degree" 로 정한다 (없으면 바닥 보기).
      z_offset : z 를 이만큼 더 올린다 [m]

      make_pose({"x": 0.4, "y": 0.1, "z": 0.3})               손목 그대로
      make_pose({"x": 0.4, "y": 0.1, "z": 0.3, "degree": 90}) 손목을 90도 돌려서
    """
    if ori is None:
        # degree 가 없으면 wrist_ori(0) == DOWN 이라 지금까지와 똑같다.
        ori = wrist_ori(pos.get("degree", 0.0))

    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = float(pos["x"])
    pose.pose.position.y = float(pos["y"])
    pose.pose.position.z = float(pos["z"]) + z_offset
    pose.pose.orientation.x = float(ori["x"])
    pose.pose.orientation.y = float(ori["y"])
    pose.pose.orientation.z = float(ori["z"])
    pose.pose.orientation.w = float(ori["w"])
    return pose
