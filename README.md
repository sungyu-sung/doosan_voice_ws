# doosan_voice_ws

두산 협동로봇(M0609)을 **말로 움직이는** ROS 2 워크스페이스입니다.
특강 *"AI가 로봇에게 귀와 입을 달아주다 — STT·TTS"* 실습용으로,
ROS 2를 처음 접하는 학생이 **YAML 파일만 고쳐서** 로봇을 다룰 수 있게 만들었습니다.

```
마이크 → 음성 인식 → 말을 명령으로 → 로봇 동작
        (STT)        (NLP)         (MoveIt2 + EGP-C 40)
```

## 들어 있는 노드

| | 노드 | 하는 일 | 로봇 필요 |
|---|---|---|:---:|
| ① | `move_node` | 좌표 한 곳으로 이동 | ○ |
| ② | `waypoint_node` | 여러 지점 순서대로 | ○ |
| ③ | `gripper_node` | 그리퍼만 열고 닫기 | — |
| ④ | `pick_place_node` | 물건 하나를 집어서 옮기기 | ○ |
| ⑤ | `gear_assembly_node` | 기어 여러 개 조립 | ○ |
| ⑥ | `stt_node` | 말을 알아듣고 글자로 | — |
| ⑦ | `nlp_node` | 글자를 로봇 명령으로 | — |
| ⑧ | `voice_robot_node` | 명령을 받아 실제로 움직이기 | ○ |
| ⑨ | `order_node` | **팀 프로젝트** — 말로 물어보고 시킨 작업 하기 | ○ |
| | `position_viewer` | 손끝 위치 보기 | — |

난이도 순서대로 배우도록 되어 있습니다. ①~⑤는 코드를 읽고 고치는 실습,
⑥~⑧은 함께 켜서 말로 로봇을 움직이는 통합 실습입니다.
⑨는 팀 프로젝트용으로, 노드 하나가 말하고 듣고 움직이는 것을 다 합니다.

## 처음 설치하는 컴퓨터에서

새 노트북에 처음 올릴 때는 아래 4개를 먼저 갖춰야 합니다.
이미 되어 있는 컴퓨터라면 [내려받기](#내려받기) 로 건너뛰세요.

### 1. MoveIt 2 (apt)

```bash
sudo apt install ros-humble-moveit
```

이게 없으면 `ModuleNotFoundError: No module named 'moveit_configs_utils'` 가 뜹니다.
경로 계획기(OMPL·Pilz)와 RViz 플러그인도 여기 들어 있습니다.

### 2. moveit_py (소스 빌드, 약 2분)

**Humble 의 apt 에는 `moveit_py` 가 없습니다.** 파이썬으로 로봇을 움직이려면
이것만 따로 빌드해야 합니다. MoveIt 전체를 빌드할 필요는 없습니다.

```bash
mkdir -p ~/moveit_py_ws/src && cd ~/moveit_py_ws/src
git clone -b humble https://github.com/ros-planning/moveit2.git

cd ~/moveit_py_ws
source /opt/ros/humble/setup.bash          # ← 다른 워크스페이스는 source 하지 말 것
colcon build --packages-select moveit_py --cmake-args -DCMAKE_BUILD_TYPE=Release
```

잘 됐는지 확인

```bash
source ~/moveit_py_ws/install/setup.bash
python3 -c "from moveit.planning import MoveItPy; print('moveit_py OK')"
```

> `moveit_py` 는 apt 로 깐 MoveIt(2.5.9)에 맞춰 빌드돼야 합니다.
> 그래서 빌드할 때 `/opt/ros/humble` 만 source 합니다.
> 다른 MoveIt 워크스페이스를 source 한 채로 빌드하면 버전이 어긋나
> `libgeometric_shapes.so...: cannot open shared object file` 같은 오류가 납니다.

> **MoveIt 튜토리얼대로 소스 빌드(`~/ws_moveit`)를 이미 하셨다면**
> 1·2번을 건너뛰고 `source ~/ws_moveit/install/setup.bash` 만 하면 됩니다.
> 그 안에 `moveit_configs_utils` 와 `moveit_py` 가 모두 들어 있습니다.
> 대신 **apt MoveIt 은 깔지 마세요.** 둘을 섞으면 깨집니다.
>
> 소스 빌드는 `sudo apt upgrade` 한 번에 깨질 수 있습니다(라이브러리 버전이
> 어긋남). 그때는 MoveIt 전체를 다시 빌드해야 해서 한 시간 넘게 걸립니다.
> 수업용으로는 1+2 방식(apt + `moveit_py` 만 소스)이 복구가 2분이라 안전합니다.

### 3. 두산 로봇 패키지

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/doosan-robotics/doosan-robot2.git
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### 4. 파이썬 라이브러리

```bash
sudo apt update
sudo apt install -y portaudio19-dev python3-dev
pip install SpeechRecognition pyaudio gtts pygame
```

`pyaudio` 는 설치할 때 소스에서 컴파일됩니다. **먼저 `portaudio19-dev` 와
`python3-dev` 를 apt 로 깔아야** 합니다. 안 그러면 아래처럼 실패합니다.

```
fatal error: portaudio.h: No such file or directory
ERROR: Could not build wheels for pyaudio
```

이 상태로 `ros2 launch voice_robot_control stt.launch.py` 를 실행하면
`speech_recognition 이 없습니다` 하고 노드가 죽습니다.
apt 로 두 패키지를 깐 뒤 `pip install` 을 다시 하면 됩니다.

### 설치 확인

```bash
source ~/doosan_voice_ws/install/setup.bash
python3 -c "import moveit_configs_utils; print('1. moveit_configs_utils OK')"
python3 -c "from moveit.planning import MoveItPy; print('2. moveit_py OK')"
ros2 pkg prefix dsr_moveit_config_m0609 && echo "3. 두산 패키지 OK"
python3 -c "import speech_recognition, gtts, pygame; print('4. 파이썬 라이브러리 OK')"
```

---

## 내려받기

처음 받을 때 — **홈 폴더에서** 받아야 `~/doosan_voice_ws` 가 됩니다.

```bash
cd ~
git clone https://github.com/YoonHJ97/doosan_voice_ws.git
cd doosan_voice_ws
```

이미 받아 놨다면, 최신 내용으로 갱신할 때

```bash
cd ~/doosan_voice_ws
git pull
```

> 내가 고친 파일이 있어서 `git pull` 이 거부되면, 내 것을 잠깐 치워두고 받은 뒤 되돌립니다.
>
> ```bash
> git stash          # 내가 고친 것 잠시 보관
> git pull
> git stash pop      # 다시 꺼내오기
> ```
>
> 내가 고친 것을 버리고 원본으로 되돌리려면
>
> ```bash
> git checkout -- src/voice_robot_control/config/keyword_map.yaml
> ```

## 한 번만 설정 — 터미널마다 source 안 하기

터미널을 열 때마다 `source` 를 치는 게 번거로우면 `~/.bashrc` 맨 아래에
아래 두 줄을 넣어 두세요. 그다음부터는 새 터미널에서 바로 쓸 수 있습니다.

```bash
# 두산 음성 로봇 워크스페이스
[ -f ~/doosan_voice_ws/install/setup.bash ] && source ~/doosan_voice_ws/install/setup.bash
```

한 줄로 넣는 명령

```bash
echo '[ -f ~/doosan_voice_ws/install/setup.bash ] && source ~/doosan_voice_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

> `[ -f ... ] &&` 를 앞에 붙인 이유는 **아직 빌드하기 전**이라도 오류가 안 나게 하기 위해서입니다.
> 이게 없으면 `colcon build` 전에는 새 터미널마다 빨간 오류가 뜹니다.

이 한 줄이 ROS 2 → `moveit_py` → 두산 패키지 → 이 워크스페이스를 **전부** 끌어옵니다.
따로 `ros2_ws` 나 `ws_moveit` 을 source 하지 마세요.

### 빌드도 한 줄로 — `dvb`

빌드할 때마다 `cd` 하고 `colcon build` 하고 다시 `source` 하는 게 번거로우면
`~/.bashrc` 맨 아래에 이 함수를 넣어 두세요.

```bash
# 두산 음성 로봇 워크스페이스 — 빌드 + source 한 번에
#   dvb                                        전체 빌드
#   dvb --packages-select voice_robot_control  한 패키지만 빌드
dvb() {
    local ws=~/doosan_voice_ws
    if [ ! -d "$ws" ]; then
        echo "$ws 가 없습니다."
        return 1
    fi
    # 빌드 전에 밑바탕부터 잡는다.
    # 이걸 안 하면 install/setup.bash 에 연결 고리가 안 적혀서
    # 나중에 source 해도 moveit_py 와 두산 패키지를 못 찾는다.
    source /opt/ros/humble/setup.bash
    [ -f ~/moveit_py_ws/install/local_setup.bash ] && source ~/moveit_py_ws/install/local_setup.bash
    [ -f ~/ros2_ws/install/local_setup.bash ]      && source ~/ros2_ws/install/local_setup.bash

    cd "$ws" || return 1
    colcon build --symlink-install "$@" || {
        echo "빌드 실패 — 위의 오류를 먼저 고치세요. (source 하지 않았습니다)"
        return 1
    }
    source "$ws/install/setup.bash"
    echo "doosan_voice_ws is built & activated!"
}
```

넣은 뒤 `source ~/.bashrc` 한 번 하면 그다음부터 `dvb` 만 치면 됩니다.

> **`source` 세 줄이 왜 빌드보다 먼저 나오나요?**
> colcon 은 **빌드할 때 잡혀 있던 것만** `install/setup.bash` 에 적어 넣습니다.
> 밑바탕을 안 잡고 빌드하면 연결 고리가 지워져서, 나중에 아무리 source 해도
> `ModuleNotFoundError: No module named 'moveit'` 이 납니다.
> 그때는 위 세 줄을 source 한 새 터미널에서 다시 빌드하면 복구됩니다.

빌드에 실패하면 **source 하지 않고 멈춥니다.** 고장 난 빌드를 source 해서
옛날 코드가 그대로 도는 것을 막기 위해서입니다.

### 그 밖에 쓸 만한 alias

```bash
alias ros2_ws="source ~/ros2_ws/install/local_setup.bash; echo \"ros2_ws is activated!\""
```

> 두산 패키지(`~/ros2_ws`)만 따로 손볼 때 씁니다.
> 평소 이 워크스페이스를 쓸 때는 **필요 없습니다** — 위의 `setup.bash` 한 줄이 다 끌어옵니다.

> **주의 — MoveIt 은 한 곳에서만 오게 하세요.**
> apt 로 깐 MoveIt 과 소스로 빌드한 MoveIt(`~/ws_moveit`)을 **둘 다 source 하면**
> 버전이 엉켜서 `move_group` 이 죽고 RViz 플러그인이 안 뜹니다.
> 위의 [처음 설치](#처음-설치하는-컴퓨터에서) 에서 고른 방식 하나만 쓰세요.
>
> 증상은 보통 이렇게 나타납니다.
>
> ```
> libgeometric_shapes.so.2.3.2: cannot open shared object file
> ```
>
> 지금 터미널에 무엇이 잡혀 있는지 보는 법
>
> ```bash
> echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -i moveit
> ```
>
> `source` 는 경로를 덧붙일 뿐 지우지 않습니다. 섞였다면 **새 터미널**을 여세요.

## 빌드 (처음 한 번만)

**빌드하기 전에 아래 것들을 먼저 source 해야 합니다.** 그래야 이 워크스페이스의
`setup.bash` 하나가 나머지를 전부 끌어오게 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/moveit_py_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash

cd ~/doosan_voice_ws
colcon build --symlink-install
```

> `setup.bash` 가 아니라 **`local_setup.bash`** 를 쓰는 이유:
> `setup.bash` 는 그 워크스페이스가 예전에 딸고 있던 것까지 함께 끌어옵니다.
> 옛 MoveIt 워크스페이스가 섞여 들어오는 것을 막으려고 `local_setup.bash` 를 씁니다.

빌드가 끝나면 이제부터는 이 한 줄이면 됩니다.

```bash
source ~/doosan_voice_ws/install/setup.bash
```

잘 잡혔는지 확인

```bash
ros2 pkg executables voice_robot_control     # 노드 9개 + position_viewer 가 보여야 정상
```

## 빠르게 해 보기

```bash
source ~/doosan_voice_ws/install/setup.bash

# 터미널 1 — 로봇 (시뮬레이터)
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609

# 터미널 2 — 말로 움직이기
ros2 launch voice_robot_control voice.launch.py
```

마이크가 없으면 타이핑으로도 됩니다.

```bash
ros2 launch voice_robot_control voice.launch.py use_stt:=false
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 2번 집어'"
```

## 팀 프로젝트 — 말로 시키는 로봇 (⑨)

물어보고, 듣고, 시킨 작업을 하는 로봇입니다. **터미널 하나**면 됩니다.

```bash
# 터미널 1 — 로봇 (시뮬레이터)
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609

# 터미널 2 — 말로 시키기
ros2 launch voice_robot_control order.launch.py
```

```
🔊 "어떤 동작을 원하시나요? 콜라, 사이다 중에 골라 주세요."
🎤 "콜라"                 → if 문이 골라내고 → 작업1_하기() 가 실행됩니다
🔊 "콜라 끝났습니다."
```

고치는 곳은 `voice_robot_control/order_node.py` 맨 위의 **작업 이름과 좌표**입니다.

```python
작업1_이름 = "콜라"                                # 이 말을 들으면 작업 1
작업2_이름 = "사이다"

좌표1 = {"x": 0.393, "y":  0.094, "z": 0.330}     # 집을 물건 위
좌표2 = {"x": 0.393, "y":  0.094, "z": 0.280}     # 집을 물건
좌표3 = {"x": 0.393, "y": -0.206, "z": 0.330}     # 놓을 자리 위
좌표4 = {"x": 0.393, "y": -0.206, "z": 0.280}     # 놓을 자리
```

**작업마다 자기 블록이 따로 있어서**, 작업 1 과 작업 2 를 아주 다른 동작으로
만들 수 있습니다. 블록은 한 줄씩 쌓듯 내려갑니다.

```python
def 작업1_하기():
    그리퍼_열기()          # 집게를 열고
    time.sleep(1.0)

    이동(좌표1)            # 물건 위로 가서
    time.sleep(1.0)

    이동(좌표2)            # 내려가서
    time.sleep(1.0)

    그리퍼_닫기()          # 잡고
    time.sleep(1.0)
    ...
```

로봇이나 마이크가 없어도 연습할 수 있습니다 (파일 맨 위 `로봇_사용 = False`,
`마이크_사용 = False`). 자세한 설명은
[패키지 README 의 ⑨ 항목](src/voice_robot_control/README.md#-order_nodepy--팀-프로젝트) 에 있습니다.

## 학생이 고치는 파일

파이썬을 건드리지 않고 두 파일만 고치면 됩니다. **다시 빌드하지 않습니다.**

| 파일 | 정하는 것 |
|---|---|
| `config/keyword_map.yaml` | 어떤 **말**을 어떤 명령으로 볼지 |
| `config/my_actions.yaml` | 우리 팀이 만드는 새 **동작** |

```yaml
# my_actions.yaml — 동작 만들기
actions:
  인사:
    - 홈
    - 이동: {x: 0.40, y:  0.15, z: 0.50}
    - 이동: {x: 0.40, y: -0.15, z: 0.50}
    - 홈

# keyword_map.yaml — 그 동작을 부를 말 붙이기
commands:
  인사:
    - 인사해
    - 안녕
```

바꾼 낱말이 잘 먹히는지는 로봇·ROS 없이 바로 확인할 수 있습니다.

```bash
python3 src/voice_robot_control/voice_robot_control/nlp_node.py "안녕"
```

## 자세한 사용법

**→ [`src/voice_robot_control/README.md`](src/voice_robot_control/README.md)**

설치·실행 방법, 노드별 실습 내용, 안전 설정, 문제 해결이 모두 정리돼 있습니다.

## 필요한 환경

- Ubuntu 22.04 / ROS 2 Humble
- MoveIt 2 + `moveit_py` — [처음 설치하는 컴퓨터에서](#처음-설치하는-컴퓨터에서) 참고
- [doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) (M0609)
- Coact(SCHUNK) EGP-C 40 그리퍼 — 로봇 플랜지 디지털 출력 1, 2번 (없어도 시늉 모드로 실습 가능)
- 인터넷 (⑥ 음성 인식, ⑨ 말하기가 구글 서버를 씁니다)
- 스피커 (⑨ 가 말합니다. 없으면 화면에 글자로만 나옵니다)

## 자주 막히는 곳

| 화면에 뜨는 말 | 무엇을 할까 |
|---|---|
| `No module named 'moveit_configs_utils'` | `sudo apt install ros-humble-moveit` |
| `No module named 'moveit'` / `MoveItPy` 없음 | `moveit_py` 를 안 빌드했습니다 → [2번](#2-moveit_py-소스-빌드-약-2분) |
| `libgeometric_shapes.so...: cannot open` | apt MoveIt 과 소스 MoveIt 이 섞였습니다. 새 터미널에서 하나만 source |
| `dsr.srdf.xacro doesn't exist` | 오래된 버전입니다. `git pull` 후 다시 빌드하세요 |
| `Package 'dsr_moveit_config_m0609' not found` | 두산 패키지가 없습니다 → [3번](#3-두산-로봇-패키지) |
| `No module named 'speech_recognition'` | `sudo apt install -y portaudio19-dev python3-dev` 후 `pip install SpeechRecognition pyaudio` |
| `fatal error: portaudio.h` / `Could not build wheels for pyaudio` | apt 로 `portaudio19-dev python3-dev` 를 먼저 깔고 다시 `pip install` |
| 그리퍼가 안 움직임 | 로봇 브링업(dsr_bringup2)이 떠 있는지, `~/ros2_ws` 를 source 했는지 확인 |

더 자세한 문제 해결은
[`src/voice_robot_control/README.md`](src/voice_robot_control/README.md) 에 있습니다.

## 라이선스

Apache-2.0
