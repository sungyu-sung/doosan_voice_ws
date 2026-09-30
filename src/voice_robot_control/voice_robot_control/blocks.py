#!/usr/bin/env python3
"""
blocks.py — 블록 하나하나 (⑨ order_node 가 쓰는 부품)

order_node.py 를 **블록코딩처럼 한 줄씩** 쓸 수 있게, 복잡한 것을 전부
여기로 숨겨 두었다.

    학생은 이 파일을 안 봐도 됩니다. order_node.py 만 고치면 됩니다.

여기 들어 있는 블록:

    준비()                맨 처음 한 번. 로봇·마이크·스피커를 켠다
    말하기("안녕하세요")   스피커로 말한다               (TTS)
    듣기()                마이크로 듣고 글자로 돌려준다   (STT)
    홈으로()              홈 자세로 간다
    이동(자리)            그 좌표로 간다
    이동(자리, 위로=0.05)  그 좌표의 5cm 위로 간다
    그리퍼(500)           집게를 50mm 로 벌린다 (물건을 놓는다)
    그리퍼(150)           집게를 15mm 로 좁힌다 (물건을 잡는다)
    그리퍼(150, 100)      좁히되 약하게 (10N)
    끝내기()              프로그램을 마친다

블록은 모두 성공하면 True, 실패하면 False 를 돌려준다.
실패해도 프로그램은 멈추지 않고 다음 줄로 넘어간다.
(수업 중에 한 곳이 안 된다고 전부 멈춰 버리면 곤란하기 때문이다)
"""

import os
import time

import rclpy
from rclpy.logging import get_logger
from std_msgs.msg import String

from .gripper_control import connect_gripper, is_gripping, wait_until_done
# 손목 각도 다루기. 순수 계산이라 MoveIt 이 없어도 불러올 수 있다.
from .pose_utils import wrap_deg
# 말하기(TTS) 는 ⑧ voice_robot_node 와 똑같은 것을 쓴다.
from .speaker import Speaker

# 로봇 팔을 움직이는 부분은 MoveIt 이 필요하다.
# MoveIt 이 없는 컴퓨터에서도 '말하고 듣기' 는 연습할 수 있도록,
# 없으면 없는 대로 넘어가고 로봇을 쓰려 할 때만 알려준다.
try:
    from .robot_common import (
        finish, go_home, make_home_state, make_plan_params, make_pose,
        plan_and_execute, setup_robot,
    )
    _무브잇있음 = True
except ImportError as _e:
    _무브잇있음 = False
    _무브잇오류 = _e


# ══════════════════════════════════════════════════════════
#  설정 — 보통은 안 건드립니다
#  (집게 폭은 order_node.py 에서 그리퍼(폭) 로 그때그때 정합니다)
# ══════════════════════════════════════════════════════════

로봇_속도 = 0.15         # 0 에 가까울수록 느림

열린_폭 = 500            # 준비() 가 처음에 벌려 두는 폭 (50mm)
                        #   ※ 단위가 1/10 mm 입니다
쥐는_힘 = 300            # 그리퍼(폭) 에 힘을 안 적었을 때 쓰는 값 (30N)
                        #   ※ 단위가 1/10 N 입니다

말하는_언어 = "ko"        # 스피커 (gTTS)
듣는_언어 = "ko-KR"       # 마이크 (구글 음성 인식)
최대_듣기_시간 = 5.0      # 한 번에 몇 초까지 들을까 [초]
소음_측정_시간 = 2.0      # 시작할 때 주변 소음을 몇 초 들어볼까 [초]

# ══════════════════════════════════════════════════════════


# ── 준비된 것들을 담아 두는 자리 ──────────────────────────
_로그 = None
_노드 = None

_로봇 = None
_팔 = None
_홈자세 = None
_홈계획 = None
_이동계획 = None
_로봇켜짐 = False

_그리퍼 = None
_진짜그리퍼 = False
_집게_폭 = 열린_폭       # 지금 얼마나 벌어져 있나. 준비() 가 열어 두고 시작한다

_인식기 = None
_마이크 = None
_우편함 = []             # /stt_result 로 들어온 말을 담아 두는 곳

_스피커 = None           # speaker.Speaker — 말하는 담당


# ══════════════════════════════════════════════════════════
#  준비
# ══════════════════════════════════════════════════════════
def 준비(로봇=True, 마이크=True):
    """
    맨 처음 한 번 부른다.

      로봇  = False → 팔·그리퍼를 안 쓰고 화면에만 찍는다 (자리에서 연습용)
      마이크 = False → 마이크 대신 /stt_result 토픽을 기다린다
    """
    global _로그, _노드, _로봇, _팔, _홈자세, _홈계획, _이동계획, _로봇켜짐
    global _그리퍼, _진짜그리퍼

    if not rclpy.ok():
        rclpy.init()

    _노드 = rclpy.create_node("order_node")
    _로그 = _노드.get_logger()

    _스피커_준비()
    _마이크_준비(마이크)

    # ── 그리퍼 (먼저 열어 둔다) ─────────────────────────
    _그리퍼, _진짜그리퍼 = connect_gripper(_로그, 로봇)
    _그리퍼.move_gripper(width_val=열린_폭, force_val=쥐는_힘)
    wait_until_done(_그리퍼, _로그)

    # ── 로봇 팔 ─────────────────────────────────────────
    if 로봇 and not _무브잇있음:
        _로그.error(
            f"MoveIt 을 불러오지 못했습니다: {_무브잇오류}\n"
            "  · 이 컴퓨터에 moveit_py 가 없습니다 (README 2번 참고)\n"
            "  → 로봇 없이 말하고 듣는 것만 연습합니다.")
        로봇 = False

    if 로봇:
        _로봇, _팔 = setup_robot(_로그, node_name="order_node_moveit")
        _홈계획, _이동계획 = make_plan_params(_로봇, vel_move=로봇_속도)
        _홈자세 = make_home_state(_로봇)
        _로봇켜짐 = True
    else:
        _로그.info("로봇: 시늉만 합니다 (로봇_사용 = False)")
        _로봇켜짐 = False

    _로그.info("준비 끝났습니다.")


# ══════════════════════════════════════════════════════════
#  말하기 (TTS)
# ══════════════════════════════════════════════════════════
def _스피커_준비():
    """gTTS + pygame 을 켠다. 안 되면 글자로만 보여준다."""
    global _스피커
    _스피커 = Speaker(_로그, 말하는_언어)
    _스피커.setup()


def 말하기(*내용, sep=" ") -> bool:
    """
    스피커로 말한다. print 처럼 쓰면 된다. 화면에도 같이 찍힌다.

      말하기("안녕하세요")
      말하기("콜라", "가져다 드릴게요")     ← 여러 개를 주면 띄어쓰기로 이어 붙인다
      말하기("남은 개수는", 3, "개입니다")  ← 숫자를 그냥 넣어도 된다
      말하기("가", "나", "다", sep="")      ← 붙여서 말하고 싶으면

    다 말할 때까지 기다린다. 안 기다리면 말하는 도중에 다음 줄로 넘어가
    로봇이 먼저 움직여 버린다.
    """
    global _로그
    문장 = sep.join(str(x) for x in 내용)

    # 준비() 를 안 부르고 말하기() 부터 써도 되게 한다.
    if _로그 is None:
        _로그 = get_logger("order_node")
    if _스피커 is None:
        _스피커_준비()

    _로그.info(f'[말] "{문장}"')
    return _스피커.say(문장)


# ══════════════════════════════════════════════════════════
#  듣기 (STT)
# ══════════════════════════════════════════════════════════
def _마이크_준비(사용: bool):
    """마이크를 연다. 못 열면 /stt_result 토픽을 기다리는 쪽으로 넘어간다."""
    global _인식기, _마이크

    if not 사용:
        _토픽으로_듣기_준비("마이크_사용 = False")
        return

    try:
        import speech_recognition as sr
    except ImportError:
        _토픽으로_듣기_준비(
            "speech_recognition 이 없습니다 "
            "(설치: sudo apt install -y portaudio19-dev python3-dev && "
            "pip install SpeechRecognition pyaudio)")
        return

    try:
        _인식기 = sr.Recognizer()
        _인식기.pause_threshold = 0.8
        _인식기.dynamic_energy_threshold = True
        _마이크 = sr.Microphone()

        _로그.info(f"주변 소음을 {소음_측정_시간:.0f}초 동안 들어봅니다. 조용히 해 주세요 ...")
        with _마이크 as source:
            _인식기.adjust_for_ambient_noise(source, duration=소음_측정_시간)
        _로그.info(f"마이크 준비 완료 (감도 {_인식기.energy_threshold:.0f})")
    except Exception as e:
        _인식기 = None
        _마이크 = None
        _토픽으로_듣기_준비(f"마이크를 열지 못했습니다: {e}")


def _토픽으로_듣기_준비(이유: str):
    """마이크 대신 /stt_result 토픽으로 말을 받는다."""
    _노드.create_subscription(
        String, "/stt_result", lambda msg: _우편함.append(msg.data), 10)
    _로그.warning(
        f"{이유}\n"
        "  → 마이크 대신 /stt_result 를 기다립니다.\n"
        "    다른 터미널에서 이렇게 주문하세요:\n"
        "      ros2 topic pub --once /stt_result std_msgs/msg/String \"data: '콜라'\"\n"
        "    또는 ⑥ 음성 인식 노드를 켜세요:\n"
        "      ros2 launch voice_robot_control stt.launch.py"
    )


def 듣기() -> str:
    """
    손님 말을 듣고 글자로 돌려준다.

    못 알아들으면 빈 글자("")를 돌려준다.
    """
    if _인식기 is None:
        return _토픽에서_듣기()

    import speech_recognition as sr

    _로그.info("듣는 중 ... (말씀하세요)")
    try:
        with _마이크 as source:
            소리 = _인식기.listen(source, phrase_time_limit=최대_듣기_시간)
    except Exception as e:
        _로그.error(f"마이크에서 소리를 못 받았습니다: {e}")
        return ""

    try:
        말 = _인식기.recognize_google(소리, language=듣는_언어)
    except sr.UnknownValueError:
        _로그.info("  (무슨 말인지 못 알아들었습니다)")
        return ""
    except sr.RequestError as e:
        _로그.error(
            f"구글 음성 인식 서버에 연결하지 못했습니다: {e}\n"
            "  · 인터넷이 연결돼 있는지 확인하세요")
        return ""

    _로그.info(f'  >>> 들은 말: "{말}"')
    return 말


def _토픽에서_듣기() -> str:
    """/stt_result 로 말이 올 때까지 기다린다."""
    _우편함.clear()
    _로그.info("/stt_result 를 기다립니다 ...")

    while rclpy.ok() and not _우편함:
        rclpy.spin_once(_노드, timeout_sec=0.2)

    if not _우편함:
        return ""

    말 = _우편함.pop(0)
    _로그.info(f'  >>> 들은 말: "{말}"')
    return 말


# ══════════════════════════════════════════════════════════
#  움직이기
# ══════════════════════════════════════════════════════════
def 홈으로() -> bool:
    """홈 자세로 간다."""
    if not _로봇켜짐:
        _로그.info("[시늉] 홈 자세로")
        return True
    return go_home(_로봇, _팔, _로그, _홈자세, _홈계획)


def 이동(자리: dict, 위로: float = 0.0) -> bool:
    """
    그 좌표로 간다. 손끝은 늘 바닥을 본다.

      이동(콜라_자리)              그 자리로
      이동(콜라_자리, 위로=0.05)   그 자리의 5cm 위로

    자리에 "degree" 를 적어 두면 손목을 그만큼 돌린 채로 간다.

      {"x": 0.4, "y": 0.1, "z": 0.3}               손목 그대로
      {"x": 0.4, "y": 0.1, "z": 0.3, "degree": 90} 손목을 90도 돌려서
    """
    if not _로봇켜짐:
        _로그.info(
            f"[시늉] 이동 → x {자리['x']:.3f}  y {자리['y']:.3f}  "
            f"z {자리['z'] + 위로:.3f}  손목 {wrap_deg(자리.get('degree', 0.0)):g}도")
        time.sleep(0.5)
        return True

    # 손목 각도(degree)는 make_pose 가 알아서 읽어 쓴다.
    성공 = plan_and_execute(
        _로봇, _팔, _로그,
        pose_goal=make_pose(자리, z_offset=위로),
        plan_parameters=_이동계획)

    if not 성공:
        _로그.error(
            "  → order_node.py 의 좌표를 확인하세요.\n"
            "    지금 손끝이 어디 있는지 보려면: "
            "ros2 launch voice_robot_control viewer.launch.py")
    return 성공


# ══════════════════════════════════════════════════════════
#  그리퍼
# ══════════════════════════════════════════════════════════
def 그리퍼(폭: int, 힘: int = None) -> bool:
    """
    집게를 그 폭으로 움직인다. 열기와 닫기가 이 함수 하나다.

      그리퍼(500)         50mm 로 벌린다 (물건을 놓는다)
      그리퍼(150)         15mm 로 좁힌다 (물건을 잡는다)
      그리퍼(150, 100)    좁히되 약하게 (10N) — 약한 물건을 잡을 때
      그리퍼(잡을_폭)      order_node.py 에 적어 둔 값으로

    ※ 단위가 1/10 입니다. 500 = 50mm, 300 = 30N.
      EGP-C 40 은 열기/닫기뿐이라 폭 325 이상이면 열고 미만이면 닫습니다. 힘은 무시됩니다.

    힘을 안 적으면 blocks.py 의 쥐는_힘 을 쓴다.

    전보다 좁아지는 쪽이면 '잡는 것' 으로 보고 물건이 잡혔는지도 확인한다.
    잡으려 했는데 못 잡았으면 False. 벌리는 쪽이면 늘 True.
    (EGP-C 40 은 파지 감지가 없어서 닫으면 늘 잡은 것으로 봅니다.)
    """
    global _집게_폭

    폭 = int(폭)
    힘 = 쥐는_힘 if 힘 is None else int(힘)

    # 벌리는 것과 좁히는 것을 구분해야 한다. 벌린 뒤에는 손에 아무것도
    # 없는 게 정상인데, 거기서 '못 잡았다' 고 하면 학생이 헷갈린다.
    좁히는_중 = 폭 < _집게_폭
    _로그.info(
        f"그리퍼 {'닫기' if 좁히는_중 else '열기'} "
        f"({폭 / 10:.0f}mm, {힘 / 10:.0f}N)")

    _그리퍼.move_gripper(width_val=폭, force_val=힘)
    wait_until_done(_그리퍼, _로그)
    _집게_폭 = 폭

    if not 좁히는_중:
        return True

    if is_gripping(_그리퍼, _로그):
        _로그.info("  · 물건을 제대로 잡았습니다.")
        return True

    _로그.warning(
        "  · 물건을 못 잡은 것 같습니다.\n"
        "      · 좌표가 물건 자리와 맞는지 확인하세요\n"
        f"      · 잡는 폭({폭 / 10:.0f}mm)이 물건보다 큰 건 아닌지 확인하세요\n"
        "    → 일단 계속 진행합니다.")
    return False


# ══════════════════════════════════════════════════════════
#  끝내기
# ══════════════════════════════════════════════════════════
def 끝내기():
    """프로그램을 깔끔하게 마친다."""
    if _진짜그리퍼:
        _그리퍼.close_connection()

    if _무브잇있음:
        # MoveIt 뒤처리 도중에 비정상 종료(-11) 하는 것을 막는다.
        # 자세한 이유는 robot_common.finish() 에 적어 두었다.
        finish(_로그)

    import sys
    _로그.info("프로그램을 마칩니다.")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
