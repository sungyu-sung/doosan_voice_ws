#!/usr/bin/env python3
"""
voice_robot_node.py — ⑧ 음성 명령 실행 노드  [Module-7 이어서]

⑦ 이 만든 명령을 받아서 **로봇을 실제로 움직인다.**
이게 붙으면 말 한 마디로 로봇이 움직이는 전체 길이 완성된다.

    마이크 → ⑥ stt_node → ⑦ nlp_node → ⑧ 이 노드 → 로봇

    받는 것:  /robot_command   (⑦ 이 보냄)
    내보내는 것: /robot_status  (지금 뭘 했는지. 다른 프로그램이 받아 쓸 수 있게)
                 + 스피커로도 같은 말을 읽어 줍니다 (USE_TTS)

    ros2 launch voice_robot_control voice_robot.launch.py

알아듣는 명령:

    home              홈 자세로
    pick 2            2번 기어 집기
    place 2           2번 기어 놓기
    pickplace 2       2번 기어 집어서 옮기기
    pickplace all     전체 기어 순서대로
    jog forward       앞으로 조금  (backward/left/right/up/down)
    open / close      그리퍼 열기 / 닫기
    stop              기다리는 명령 모두 취소

■ 마이크 없이 시험하기
      ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'pick 2'"

■ 실습
  · GEAR_TASKS 좌표를 우리 작업대에 맞게 고쳐 보세요.
  · degree 를 넣으면 집게를 그만큼 돌려서 집고 놓습니다.
  · JOG_STEP 을 바꿔 "앞으로" 한 번에 얼마나 갈지 조절해 보세요.

  my_actions.yaml 의 '이동' 에도 degree 를 넣을 수 있습니다.
      - 이동: {x: 0.4, y: 0.1, z: 0.3, degree: 90}
"""

import os
import queue
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import get_logger
from std_msgs.msg import String

from .gripper_control import connect_gripper, is_gripping, wait_until_done
from .robot_common import (
    current_tcp, go_home, make_home_state, make_joint_state, make_plan_params,
    make_pose, plan_and_execute, setup_robot,
)
# 말하기(TTS) 는 ⑨ blocks.py 와 똑같은 것을 쓴다.
from .speaker import Speaker


# ══════════════════════════════════════════════════════════
#  ▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼
# ══════════════════════════════════════════════════════════

# 기어를 어디서 집어 어디에 놓을지 (base_link 기준, 단위 m)
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

# "앞으로" 한 번에 움직이는 거리 [m]
JOG_STEP = 0.05

# 한 일을 스피커로 읽어 줄까?
#   True  = 소리로 말합니다 (gtts, pygame 이 없으면 알려주고 글자로만)
#   False = /robot_status 토픽과 화면에만 남깁니다
USE_TTS = True
TTS_LANG = "ko"         # 영어로 하려면 "en"

# 그리퍼
USE_GRIPPER = True
OPEN_WIDTH = 500        # 열었을 때 (50mm)
CLOSE_WIDTH = 150       # 닫았을 때 (15mm)
FORCE = 300             # 쥐는 힘 (30N)

# 로봇 속도 (0에 가까울수록 느림)
SPEED = 0.15

# ══════════════════════════════════════════════════════════


# 방향 이름 → 어느 쪽으로 갈지
JOG_DIRECTIONS = {
    "forward":  (+1, 0, 0),
    "backward": (-1, 0, 0),
    "left":     (0, +1, 0),
    "right":    (0, -1, 0),
    "up":       (0, 0, +1),
    "down":     (0, 0, -1),
}


def load_actions(logger):
    """my_actions.yaml — 우리 팀이 만든 동작을 읽어온다."""
    path = os.path.join(
        get_package_share_directory("voice_robot_control"),
        "config", "my_actions.yaml")

    if not os.path.isfile(path):
        logger.warn(f"동작 파일이 없습니다: {path}")
        return {}

    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        mark = getattr(e, "problem_mark", None)
        where = f" ({mark.line + 1}째 줄 근처)" if mark else ""
        logger.error(
            f"my_actions.yaml 을 읽지 못했습니다{where}.\n"
            "  · 줄 앞의 '- ' 를 빠뜨리지 않았는지\n"
            "  · 띄어쓰기(들여쓰기) 칸수가 위아래와 같은지\n"
            f"  [자세한 내용] {e}")
        return {}

    actions = {str(k): (v or []) for k, v in (data.get("actions") or {}).items()}
    if actions:
        logger.info(
            f"우리 팀 동작 {len(actions)}개를 읽었습니다: "
            f"{', '.join(sorted(actions))}")
    return actions


class VoiceRobot:
    """명령을 받아 로봇을 움직이는 사람."""

    def __init__(self, node, logger):
        self.node = node
        self.log = logger

        # 그리퍼 먼저 (열어 둔다)
        self.gripper, self.is_real = connect_gripper(logger, USE_GRIPPER)
        self.gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
        wait_until_done(self.gripper, logger)

        # 로봇 팔
        self.robot, self.arm = setup_robot(logger, node_name="voice_robot_moveit")
        self.home_params, self.pilz_params = make_plan_params(
            self.robot, vel_move=SPEED)
        self.home_state = make_home_state(self.robot)

        self.actions = load_actions(logger)
        self.status_pub = node.create_publisher(String, "/robot_status", 10)

        # 말하기 준비. 소리를 못 내도 실습은 그대로 진행된다.
        self.speaker = Speaker(logger, TTS_LANG) if USE_TTS else None
        if self.speaker is None:
            logger.info("USE_TTS = False — /robot_status 토픽으로만 알립니다.")
        elif self.speaker.setup():
            logger.info("한 일을 스피커로 읽어 줍니다.")

        # 명령을 줄 세운다. 로봇은 한 번에 하나만 할 수 있다.
        self.jobs = queue.Queue()
        threading.Thread(target=self._worker, daemon=True).start()

        # 말도 따로 줄을 세워 다른 스레드에서 읽는다.
        #
        # 로봇을 움직이는 스레드가 직접 말하게 하면, gTTS 가 구글 서버에서
        # 소리를 받아오는 1~2초 동안 팔이 멈춰 서서 기다린다. "전체 조립"
        # 처럼 기어마다 한 마디씩 하는 경우엔 그 시간이 다 쌓인다.
        # 그래서 말은 여기에 던져만 두고 로봇은 바로 다음 동작으로 넘어간다.
        self.speech = queue.Queue()
        threading.Thread(target=self._speech_worker, daemon=True).start()

    # ── 명령 받기 ─────────────────────────────────────
    def on_command(self, msg: String):
        text = msg.data.strip().lower()
        if not text:
            return

        # 정지는 줄을 서지 않고 바로 처리한다
        if text == "stop":
            dropped = self._drain(self.jobs)
            # 취소한 일을 계속 읽어 주면 안 되니 하려던 말도 같이 버린다.
            # (이미 말하는 중인 한 마디는 끝까지 간다)
            self._drain(self.speech)
            self.say(f"정지 — 기다리던 명령 {dropped}개를 취소했습니다.")
            self.log.warn(
                "정지했습니다.\n"
                "  (이미 시작한 동작은 끝까지 갑니다. 급하면 비상정지를 누르세요)")
            return

        self.jobs.put(text)
        waiting = self.jobs.qsize()
        if waiting > 1:
            self.log.info(f"줄 세웠습니다 — 앞에 {waiting - 1}개 있음")

    @staticmethod
    def _drain(q) -> int:
        """줄에 서 있는 것을 다 버리고, 몇 개를 버렸는지 돌려준다."""
        dropped = 0
        while not q.empty():
            try:
                q.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        return dropped

    # ── 줄 서 있는 말을 하나씩 읽어 준다 ──────────────
    def _speech_worker(self):
        while rclpy.ok():
            try:
                text = self.speech.get(timeout=1.0)
            except queue.Empty:
                continue

            # 말하다 문제가 생겨도 로봇은 계속 움직여야 한다.
            try:
                self.speaker.say(text)
            except Exception as e:
                self.log.warn(f"말하지 못했습니다: {e}")

    # ── 줄에서 하나씩 꺼내 실행 ───────────────────────
    def _worker(self):
        while rclpy.ok():
            try:
                text = self.jobs.get(timeout=1.0)
            except queue.Empty:
                continue

            self.log.info(f"===== 실행: {text} =====")
            try:
                ok = self.run(text)
            except Exception as e:
                self.log.error(f"실행 중 문제가 생겼습니다: {e}")
                self.say(f"{text} 실패")
                continue

            result = f"{text} 완료" if ok else f"{text} 실패"
            self.log.info(f">>> {result}")
            self.say(result)

    # ── 명령별 동작 ───────────────────────────────────
    def run(self, text: str) -> bool:
        parts = text.split()
        head = parts[0]
        rest = parts[1] if len(parts) > 1 else None

        if head == "home":
            return self.go_home()

        if head == "open":
            return self.grip(OPEN_WIDTH, "열기")

        if head == "close":
            return self.grip(CLOSE_WIDTH, "닫기")

        if head == "jog":
            return self.jog(rest)

        if head in ("pick", "place", "pickplace"):
            return self.gear_job(head, rest)

        # 우리 팀이 my_actions.yaml 에 만든 동작
        if head in self.actions:
            return self.run_action(head)

        known = ", ".join(sorted(self.actions)) or "(없음)"
        self.log.error(
            f"'{text}' 는 모르는 명령입니다.\n"
            f"  · 기본 명령: home / pick N / place N / pickplace N / jog / open / close\n"
            f"  · 우리 팀 동작: {known}")
        return False

    # ── 우리 팀이 만든 동작 ───────────────────────────
    def run_action(self, name: str) -> bool:
        """my_actions.yaml 에 적어둔 순서대로 하나씩 실행한다."""
        steps = self.actions[name]
        total = len(steps)
        self.log.info(f"우리 팀 동작 '{name}' — 모두 {total}단계")

        for i, step in enumerate(steps, start=1):
            self.log.info(f"  [{i}/{total}] {_describe(step)}")
            if not self.run_step(name, i, step):
                return False
        return True

    def run_step(self, action_name, index, step) -> bool:
        """동작 한 줄을 실행한다."""
        # "홈" 처럼 값이 없는 줄
        if isinstance(step, str):
            kind, value = step.strip(), None
        # "이동: {...}" 처럼 값이 있는 줄
        elif isinstance(step, dict) and len(step) == 1:
            (kind, value), = step.items()
            kind = str(kind).strip()
        else:
            self.log.error(
                f"'{action_name}' {index}번째 줄을 알아볼 수 없습니다: {step}")
            return False

        if kind == "홈":
            return self.go_home()

        if kind == "열기":
            return self.grip(OPEN_WIDTH, "열기")

        if kind == "닫기":
            return self.grip(CLOSE_WIDTH, "닫기")

        if kind == "기다리기":
            time.sleep(float(value or 0))
            return True

        if kind == "이동":
            if not isinstance(value, dict) or not all(k in value for k in "xyz"):
                self.log.error(f"'이동' 은 x, y, z 가 모두 필요합니다: {value}")
                return False
            # degree 는 안 적어도 된다 (안 적으면 0).
            return self.move(value)

        if kind == "관절":
            if not isinstance(value, (list, tuple)) or len(value) != 6:
                self.log.error(f"'관절' 은 각도 6개가 필요합니다: {value}")
                return False
            state = make_joint_state(
                self.robot,
                {f"joint_{i}": v for i, v in enumerate(value, start=1)},
                self.log)
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(robot_state=state)
            return plan_and_execute(self.robot, self.arm, self.log,
                                    plan_parameters=self.home_params)

        if kind in ("집기", "놓기"):
            what = "pick" if kind == "집기" else "place"
            return self.gear_job(what, str(value))

        self.log.error(
            f"'{action_name}' {index}번째 줄의 '{kind}' 는 모르는 동작입니다.\n"
            "  쓸 수 있는 것: 홈 / 이동 / 관절 / 집기 / 놓기 / 열기 / 닫기 / 기다리기")
        return False

    def go_home(self) -> bool:
        return go_home(self.robot, self.arm, self.log,
                       self.home_state, self.home_params)

    def grip(self, width, what) -> bool:
        self.log.info(f"그리퍼 {what} ({width / 10:.0f}mm)")
        self.gripper.move_gripper(width_val=width, force_val=FORCE)
        wait_until_done(self.gripper, self.log)
        return True

    def jog(self, direction) -> bool:
        """지금 위치에서 그 방향으로 조금 움직인다."""
        if direction not in JOG_DIRECTIONS:
            self.log.error(f"'{direction}' 은 모르는 방향입니다.")
            return False

        pos, ori = current_tcp(self.robot, self.log)
        if pos is None:
            return False

        dx, dy, dz = JOG_DIRECTIONS[direction]
        target = {"x": pos["x"] + dx * JOG_STEP,
                  "y": pos["y"] + dy * JOG_STEP,
                  "z": pos["z"] + dz * JOG_STEP}

        self.log.info(
            f"{direction} 로 {JOG_STEP * 100:.0f}cm  "
            f"({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f}) → "
            f"({target['x']:.3f}, {target['y']:.3f}, {target['z']:.3f})")

        return plan_and_execute(self.robot, self.arm, self.log,
                                pose_goal=make_pose(target, ori),
                                plan_parameters=self.pilz_params)

    def gear_job(self, what, which) -> bool:
        """pick / place / pickplace 를 기어 번호에 대해 실행."""
        if which is None:
            self.log.error("몇 번 기어인지 알려주세요.")
            return False

        # 어느 기어들을 할지 정한다
        if which == "all":
            if what != "pickplace":
                self.log.error("'전체' 는 집어서 옮기기(pickplace) 에만 됩니다.")
                return False
            targets = list(range(len(GEAR_TASKS)))
        else:
            try:
                number = int(which)
            except ValueError:
                self.log.error(f"'{which}' 는 번호가 아닙니다.")
                return False
            if not 1 <= number <= len(GEAR_TASKS):
                self.log.error(f"기어 번호는 1 ~ {len(GEAR_TASKS)} 사이여야 합니다.")
                return False
            targets = [number - 1]

        total = len(targets)
        for n, idx in enumerate(targets, start=1):
            task = GEAR_TASKS[idx]
            if total > 1:
                self.log.info(f"--- {n}/{total} 번째 기어 ---")
                self.say(f"{n}번째 기어 작업 중")

            if what in ("pick", "pickplace"):
                if not self.do_pick(task["pick"]):
                    return False
            if what in ("place", "pickplace"):
                if not self.do_place(task["place"]):
                    return False

        return True

    def do_pick(self, spot) -> bool:
        self.log.info("  · 집는 자리 위로")
        if not self.move(spot, APPROACH_OFFSET):
            return False

        self.log.info("  · 집는 자리로 내려가기")
        if not self.move(spot):
            return False

        self.log.info(f"  · 그리퍼 닫기 ({CLOSE_WIDTH / 10:.0f}mm)")
        self.gripper.move_gripper(width_val=CLOSE_WIDTH, force_val=FORCE)
        wait_until_done(self.gripper, self.log)
        if not is_gripping(self.gripper, self.log):
            self.log.warning("  · 물건을 못 잡은 것 같습니다 — 계속 진행합니다.")

        self.log.info("  · 들어 올리기")
        return self.move(spot, APPROACH_OFFSET)

    def do_place(self, spot) -> bool:
        self.log.info("  · 놓는 자리 위로")
        if not self.move(spot, APPROACH_OFFSET):
            return False

        self.log.info("  · 놓는 자리로 내려가기")
        if not self.move(spot):
            return False

        self.log.info(f"  · 그리퍼 열기 ({OPEN_WIDTH / 10:.0f}mm)")
        self.gripper.move_gripper(width_val=OPEN_WIDTH, force_val=FORCE)
        wait_until_done(self.gripper, self.log)

        self.log.info("  · 빠져나오기")
        return self.move(spot, APPROACH_OFFSET)

    def move(self, spot, z_offset=0.0) -> bool:
        # spot 에 degree 가 있으면 make_pose 가 알아서 손목을 돌려준다.
        return plan_and_execute(self.robot, self.arm, self.log,
                                pose_goal=make_pose(spot, z_offset=z_offset),
                                plan_parameters=self.pilz_params)

    def say(self, text: str):
        """
        지금 뭘 했는지 알린다. 토픽으로도 내보내고 스피커로도 읽어 준다.

        여기서 기다리지 않는다. 말은 줄에 던져만 두고 바로 돌아온다.
        (로봇을 움직이는 쪽에서 부르기 때문이다 — _speech_worker 참고)
        """
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

        if self.speaker is not None:
            self.speech.put(text)


def _describe(step) -> str:
    """동작 한 줄을 사람이 읽기 좋게."""
    if isinstance(step, str):
        return step
    if isinstance(step, dict) and len(step) == 1:
        (k, v), = step.items()
        return f"{k}: {v}"
    return str(step)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("voice_robot_node")
    logger = get_logger("voice_robot_node")

    logger.info("=== ⑧ 음성 명령 실행 시작 ===")

    worker = VoiceRobot(node, logger)
    node.create_subscription(String, "/robot_command", worker.on_command, 10)

    logger.info(
        "\n"
        "════════════════════════════════════════\n"
        "  /robot_command 를 기다립니다\n"
        "  ⑥ stt_node 와 ⑦ nlp_node 도 켜져 있어야\n"
        "  말로 움직일 수 있습니다.\n"
        "  끝내려면 Ctrl+C\n"
        "════════════════════════════════════════")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if worker.is_real:
        worker.gripper.close_connection()

    logger.info("=== ⑧ 음성 명령 실행 끝 ===")
    from .robot_common import finish
    finish(logger)


if __name__ == "__main__":
    main()
