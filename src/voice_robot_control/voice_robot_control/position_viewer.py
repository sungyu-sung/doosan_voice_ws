#!/usr/bin/env python3
"""
position_viewer.py — 로봇 손끝이 어디 있는지 보여주는 노드  [Module-5]

로봇의 손끝(TCP)이 지금 어느 위치에 있는지 계속 알려준다.
로봇을 움직이지는 않으므로 켜 두어도 안전하다.

이 노드가 필요한 이유:
  poses.yaml 에 적을 좌표를 **눈으로 확인**하기 위해서다.
  로봇을 원하는 자리에 옮긴 뒤 화면의 숫자를 그대로 옮겨 적으면 된다.
  '저장' 기능을 쓰면 그 일을 자동으로 해 준다.

    구독:  /joint_states   관절 각도 (있으면 같이 보여줌)
           /record_pose    지금 위치를 이름 붙여 파일에 저장
    발행:  /tcp_position   지금 위치 (다른 프로그램이 쓰기 좋은 형태)
           /tcp_info       지금 위치 (사람이 읽기 좋은 글자)

보통은 리모컨(robot_console)에서 `where` 와 `save 이름` 으로 쓴다.
"""

import math
import os
from datetime import datetime

import rclpy
import tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .core.pose_utils import quat_to_rpy_deg, quat_to_wrist_deg


def _n(value: float, digits: int) -> str:
    """
    숫자를 보기 좋게 다듬는다.

    그냥 찍으면 0 이 '-0' 으로 나오는 경우가 있는데, 학생 눈에는
    고장난 것처럼 보인다. 아주 작은 값은 0 으로 맞춰준다.
    """
    rounded = round(value, digits)
    if rounded == 0:
        rounded = 0.0
    return f'{rounded:+.{digits}f}'


class PositionViewer(Node):

    def __init__(self):
        super().__init__('position_viewer')

        # ── 설정값 ────────────────────────────────────────
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_link', 'link_6')
        self.declare_parameter('print_period', 1.0)  # 화면에 찍는 주기 [초], 0 이면 안 찍음
        self.declare_parameter('record_file', '')    # 비우면 ~/내가_저장한_자세.yaml

        self._base_frame = self.get_parameter('base_frame').value
        self._ee_link = self.get_parameter('ee_link').value
        self._print_period = float(self.get_parameter('print_period').value)

        record_file = self.get_parameter('record_file').value
        self._record_file = record_file or os.path.join(
            os.path.expanduser('~'), '내가_저장한_자세.yaml')

        # ── TF 듣기 준비 ──────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._last_pose = None
        self._joint_deg = {}
        self._last_print = 0.0
        self._waiting_logged = False

        # ── 주고받는 토픽 ─────────────────────────────────
        self._pose_pub = self.create_publisher(PoseStamped, '/tcp_position', 10)
        self._info_pub = self.create_publisher(String, '/tcp_info', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(String, '/record_pose', self._on_record, 10)

        self.create_timer(0.1, self._tick)  # 0.1초마다 위치 확인

        self.get_logger().info(
            '\n'
            '════════════════════════════════════════\n'
            '  위치 보기 준비 완료\n'
            f'  저장 파일: {self._record_file}\n'
            '════════════════════════════════════════')

    # ════════════════════════════════════════════════
    #   0.1초마다 위치 확인해서 알려주기
    # ════════════════════════════════════════════════
    def _tick(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_link, rclpy.time.Time())
        except tf2_ros.TransformException:
            if not self._waiting_logged:
                self.get_logger().warn(
                    '로봇에게서 아직 아무 소식이 없습니다.\n'
                    '  로봇(또는 시뮬레이터)이 켜져 있는지 확인하세요.')
                self._waiting_logged = True
            return

        if self._waiting_logged:
            self.get_logger().info('로봇을 찾았습니다. 위치를 보여줍니다.')
            self._waiting_logged = False

        t, q = tf.transform.translation, tf.transform.rotation

        pose = PoseStamped()
        pose.header.stamp = tf.header.stamp
        pose.header.frame_id = self._base_frame
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = q
        self._last_pose = pose
        self._pose_pub.publish(pose)

        info = self._describe(t, q)
        msg = String()
        msg.data = info
        self._info_pub.publish(msg)

        self._maybe_print(info)

    def _describe(self, t, q) -> str:
        roll, pitch, yaw = quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        # 손목 각도 = 노드 파일의 degree 에 그대로 적을 수 있는 값
        wrist = quat_to_wrist_deg(q.x, q.y, q.z, q.w)
        text = (f'손끝 위치   x={_n(t.x, 3)}  y={_n(t.y, 3)}  z={_n(t.z, 3)}  [m]\n'
                f'  손끝 방향   {_n(roll, 0)} / {_n(pitch, 0)} / {_n(yaw, 0)}  [도]\n'
                f'  손목 각도   degree={_n(wrist, 0)}  [도]')
        if self._joint_deg:
            joints = '  '.join(
                f'J{i}={_n(self._joint_deg[f"joint_{i}"], 0)}'
                for i in range(1, 7) if f'joint_{i}' in self._joint_deg)
            text += f'\n  관절 각도   {joints}  [도]'
        return text

    def _maybe_print(self, info: str):
        if self._print_period <= 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_print < self._print_period:
            return
        self._last_print = now
        self.get_logger().info(info)

    def _on_joints(self, msg: JointState):
        for name, radians in zip(msg.name, msg.position):
            self._joint_deg[name] = math.degrees(radians)

    # ════════════════════════════════════════════════
    #   지금 위치를 파일에 저장
    # ════════════════════════════════════════════════
    def _on_record(self, msg: String):
        name = msg.data.strip()
        if not name:
            self.get_logger().error('저장할 이름이 없습니다.')
            return

        if self._last_pose is None:
            self.get_logger().error(
                '아직 로봇 위치를 모릅니다. 로봇이 켜져 있는지 확인하세요.')
            return

        p = self._last_pose.pose.position
        q = self._last_pose.pose.orientation
        wrist = quat_to_wrist_deg(q.x, q.y, q.z, q.w)
        stamp = datetime.now().strftime('%m월 %d일 %H:%M')
        # degree 까지 적어 두면 노드 파일에 그대로 복사해 쓸 수 있다.
        한줄 = (f'{name}: {{x: {p.x:.3f}, y: {p.y:.3f}, z: {p.z:.3f}, '
               f'degree: {wrist:.0f}}}')
        block = f'  # {stamp} 에 저장\n  {한줄}\n'

        try:
            first_time = not os.path.exists(self._record_file)
            with open(self._record_file, 'a', encoding='utf-8') as f:
                if first_time:
                    f.write('# 로봇 위치를 저장한 파일입니다.\n'
                            '# 아래 줄을 poses.yaml 의 tcp_poses: 밑에 복사해 넣으세요.\n'
                            'tcp_poses:\n')
                f.write(block)
        except Exception as e:
            self.get_logger().error(f'파일에 저장하지 못했습니다: {e}')
            return

        self.get_logger().info(
            f"'{name}' 저장 완료 → {self._record_file}\n  {한줄}")


def main(args=None):
    rclpy.init(args=args)
    node = PositionViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
