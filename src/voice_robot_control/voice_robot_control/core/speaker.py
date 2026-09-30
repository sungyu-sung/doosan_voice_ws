#!/usr/bin/env python3
"""
speaker.py — 글자를 소리로 바꿔 말하기 (TTS)

gTTS 로 구글 서버에서 mp3 를 받아 pygame 으로 재생한다.
⑧ voice_robot_node 와 ⑨ blocks.py 가 **똑같은 방법으로** 말하도록
여기 한 곳에 모아 두었다.

소리를 못 내는 컴퓨터에서도 실습이 멈추면 안 되므로,
gtts/pygame 이 없거나 인터넷이 안 되면 경고만 하고 그냥 넘어간다.
(말한 문장을 화면에 찍는 것은 부르는 쪽에서 한다)
"""

import hashlib
import os
import tempfile
import time

# pygame 이 시작할 때 찍는 광고 문구를 감춘다. import 보다 먼저 해야 한다.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "hide")

DEFAULT_LANG = "ko"      # 말하는 언어 (영어로 하려면 "en")


class Speaker:
    """
    말하는 담당.

        speaker = Speaker(logger)
        speaker.setup()              # 한 번만. 안 되면 False
        speaker.say("안녕하세요")     # 다 말할 때까지 기다린다

    setup() 이 실패했어도 say() 를 불러도 된다. 아무 소리도 내지 않고
    False 를 돌려줄 뿐이라, 부르는 쪽에 if 문을 늘리지 않아도 된다.
    """

    def __init__(self, logger, lang=DEFAULT_LANG):
        self.log = logger
        self.lang = lang
        self.ready = False       # 소리를 낼 수 있는 상태인가
        self.tried = False       # setup() 을 한 번이라도 해 봤나

    # ── 준비 ──────────────────────────────────────────
    def setup(self) -> bool:
        """gTTS + pygame 을 켠다. 안 되면 글자로만 보여주게 둔다."""
        self.tried = True
        try:
            import pygame
            from gtts import gTTS      # noqa: F401  (있는지만 확인)

            pygame.mixer.init()
            self.ready = True
        except Exception as e:
            self.ready = False
            self.log.warning(
                f"소리를 낼 수 없어 글자로만 보여줍니다: {e}\n"
                "  설치: pip install gtts pygame\n"
                "  → 실습은 그대로 진행됩니다."
            )
        return self.ready

    # ── 소리 파일 만들기 ───────────────────────────────
    def sound_file(self, text: str) -> str:
        """
        문장을 mp3 로 만들어 그 파일 경로를 돌려준다.

        한 번 만든 문장은 저장해 두고 다시 쓴다. 같은 인사말을 손님마다
        구글 서버에서 다시 받아오면 그때마다 1~2초씩 멈추기 때문이다.
        """
        folder = os.path.join(tempfile.gettempdir(), "voice_robot_tts")
        os.makedirs(folder, exist_ok=True)

        name = hashlib.md5(f"{self.lang}:{text}".encode("utf-8")).hexdigest()
        path = os.path.join(folder, f"{name}.mp3")

        if not os.path.exists(path):
            from gtts import gTTS
            gTTS(text=text, lang=self.lang).save(path)

        return path

    # ── 말하기 ────────────────────────────────────────
    def say(self, text: str) -> bool:
        """
        한 문장을 소리로 낸다. 다 말할 때까지 기다린다.

        기다리는 이유: 안 기다리면 말하는 도중에 다음 문장을 덮어써서
        앞말이 잘린다. (로봇이 기다리면 안 되는 곳에서는 부르는 쪽이
        따로 스레드를 두고 여기를 부른다 — voice_robot_node 가 그렇게 한다)
        """
        # setup() 을 안 부르고 say() 부터 써도 되게 한다.
        if not self.tried:
            self.setup()

        if not self.ready or not text:
            return False

        try:
            import pygame
            pygame.mixer.music.load(self.sound_file(text))
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                time.sleep(0.1)

            if hasattr(pygame.mixer.music, "unload"):
                pygame.mixer.music.unload()
            return True
        except Exception as e:
            self.log.warning(
                f"말하지 못했습니다: {e}\n"
                "  · 인터넷이 연결돼 있는지 확인하세요 (구글 서버에서 소리를 받아옵니다)\n"
                "  · 스피커가 연결돼 있는지 확인하세요"
            )
            return False
