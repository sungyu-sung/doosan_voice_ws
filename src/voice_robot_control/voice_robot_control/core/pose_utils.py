#!/usr/bin/env python3
"""
pose_utils.py — 자세(orientation) 변환 유틸

pose_mover_node 와 tcp_monitor_node 가 **같은 변환 규칙**을 쓰도록 여기 모아둔다.
이게 중요한 이유:
  tcp_monitor 가 화면에 찍어준 roll/pitch/yaw 값을 학생이 그대로 poses.yaml 에
  복사해 넣으면 정확히 같은 자세로 돌아가야 하기 때문이다.

회전 순서는 ROS 표준인 ZYX (yaw → pitch → roll) 고정.
각도 단위는 사람이 읽기 쉬운 **degree** 를 쓴다.

참고:
  roll=180, pitch=0, yaw=180  →  쿼터니언 (x,y,z,w) = (0, 1, 0, 0)
  = 그리퍼가 바닥(-z)을 바라보는 기본 작업 자세
"""

import math


def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple:
    """RPY[deg] → 쿼터니언 (x, y, z, w).  회전 순서 ZYX."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y_, z, w


def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> tuple:
    """쿼터니언 (x, y, z, w) → RPY[deg].  rpy_deg_to_quat 의 역변환."""
    # 회전행렬 성분 중 필요한 것만 계산
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + z * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    # 짐벌락 근처에서 asin 정의역을 벗어나지 않도록 클램프
    r20 = max(-1.0, min(1.0, r20))

    roll = math.atan2(r21, r22)
    pitch = math.asin(-r20)
    yaw = math.atan2(r10, r00)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wrap_deg(degree: float) -> float:
    """각도를 -180 ~ 180 안으로 접어 넣는다. (270 → -90, 같은 방향이다)"""
    return (float(degree) + 180.0) % 360.0 - 180.0


def wrist_deg_to_quat(degree: float = 0.0) -> tuple:
    """
    손목만 degree[도] 돌린 쿼터니언 (x, y, z, w).

    손끝은 늘 바닥(-z)을 보고, 그 상태에서 제자리 회전만 한다.
      degree = 0    →  (0, 1, 0, 0) = 지금까지의 기본 작업 자세와 같다
      degree = 90   →  집게를 90도 돌려서

    ±180 을 넘는 값은 같은 방향인 값으로 접어서 쓴다. (270 = -90)
    """
    q = rpy_deg_to_quat(180.0, 0.0, 180.0 + wrap_deg(degree))
    # cos(90도) 가 0 이 아니라 6e-17 로 나온다. 로봇에는 아무 영향이 없지만,
    # degree=0 일 때 robot_common.DOWN 과 한 글자도 다르지 않게 해 두면
    # "degree 0 은 지금까지와 똑같다" 를 눈으로 확인할 수 있다.
    return tuple(0.0 if abs(v) < 1e-12 else v for v in q)


def quat_to_wrist_deg(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → 손목 각도[도].  wrist_deg_to_quat 의 역변환."""
    _, _, yaw = quat_to_rpy_deg(x, y, z, w)
    return wrap_deg(yaw - 180.0)


def normalize_quat(x: float, y: float, z: float, w: float) -> tuple:
    """쿼터니언 정규화. 길이가 0이면 단위 쿼터니언을 돌려준다."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def parse_orientation(spec: dict) -> tuple:
    """
    poses.yaml 의 자세 표기를 쿼터니언 (x, y, z, w) 로 변환.

    허용하는 표기 3가지:
      1) roll / pitch / yaw   (deg)   ← 권장
      2) qx / qy / qz / qw            ← 쿼터니언 직접 지정
      3) 생략                          ← 기본값 (그리퍼가 바닥을 봄)
    """
    if any(k in spec for k in ('qx', 'qy', 'qz', 'qw')):
        return normalize_quat(
            float(spec.get('qx', 0.0)),
            float(spec.get('qy', 0.0)),
            float(spec.get('qz', 0.0)),
            float(spec.get('qw', 1.0)),
        )

    if any(k in spec for k in ('roll', 'pitch', 'yaw')):
        return rpy_deg_to_quat(
            float(spec.get('roll', 0.0)),
            float(spec.get('pitch', 0.0)),
            float(spec.get('yaw', 0.0)),
        )

    # 기본 작업 자세: 그리퍼가 바닥을 향함
    return rpy_deg_to_quat(180.0, 0.0, 180.0)
