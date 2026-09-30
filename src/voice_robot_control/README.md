# voice_robot_control — 파이썬으로 로봇 움직이기

특강 **Module-5 / 6 / 7** 용 패키지입니다. `dsr_practice` 의 코드를 바탕으로,
수업에서 바로 고쳐 쓸 수 있게 다시 정리했습니다.

## 노드 9개 + 1개

| 노드 | 하는 일 | 로봇 팔 | 그리퍼 |
|------|---------|:---:|:---:|
| ① `move_node` | 홈으로 갔다가 좌표 한 곳으로 이동 | ○ | — |
| ①-2 `joint_move_node` | 관절 각도를 직접 정해서 이동 | ○ | — |
| ② `waypoint_node` | 여러 지점을 순서대로 지나가기 | ○ | — |
| ③ `gripper_node` | 집게만 열고 닫기 | — | ○ |
| ④ `pick_place_node` | 물건 하나를 집어서 옮기기 | ○ | ○ |
| ⑤ `gear_assembly_node` | 기어 여러 개를 순서대로 조립 | ○ | ○ |
| ⑥ `stt_node` | 말을 알아듣고 글자로 바꾸기 | — | — |
| ⑦ `nlp_node` | 글자를 로봇 명령으로 바꾸기 | — | — |
| ⑧ `voice_robot_node` | **명령을 받아 로봇을 움직이고 한 일을 말해 주기** | ○ | ○ |
| ⑨ `order_node` | **팀 프로젝트 — 말로 물어보고 시킨 작업 하기** | ○ | ○ |
| `position_viewer` | 손끝이 지금 어디 있는지 보여주기 | — | — |

**①→②→③→④→⑤ 순서로 배웁니다.**
①을 여러 번 이어 붙이면 ②, ①과 ③을 합치면 ④, ④를 여러 번 반복하면 ⑤ 입니다.
⑥⑦⑧ 을 함께 켜면 **말 한 마디로 로봇이 움직입니다.**

```
마이크 → ⑥ stt_node → /stt_result → ⑦ nlp_node → /robot_command → ⑧ voice_robot_node → 로봇
```

⑨ 는 **팀 프로젝트용**입니다. 위 셋을 나눠 켤 필요 없이 **혼자서 말하고 듣고
움직입니다.** 노드 하나만 켜면 되고, 코드는 블록을 쌓듯 한 줄씩 내려갑니다.

---

## 1. 빌드 (처음 한 번만)

```bash
cd ~/doosan_voice_ws
colcon build --symlink-install
```

> `--symlink-install` 덕분에 **`.py` 파일을 고친 뒤 다시 빌드하지 않아도 됩니다.**
> 고치고 바로 다시 실행하면 반영됩니다.

## 2. 실행

**터미널마다 이 한 줄을 먼저 칩니다.**

```bash
source ~/doosan_voice_ws/install/setup.bash
```

> **`ws_moveit` 은 source 하지 마세요.** 옛날 라이브러리로 빌드돼 있어서
> 정상 MoveIt 을 가려 버립니다. `move_group` 이 죽거나 RViz 플러그인이
> 안 뜨면 이걸 의심하세요. 확인 방법:
> ```bash
> echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep ws_moveit   # 아무것도 안 나와야 정상
> ```

**터미널 1 — 로봇 켜기**  (③ 그리퍼 노드만 쓸 때는 생략 가능)

```bash
# 시뮬레이터
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609

# 실제 로봇
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real model:=m0609 host:=192.168.137.100
```

**터미널 2 — 하고 싶은 것 하나 고르기**

```bash
ros2 launch voice_robot_control move.launch.py       # ① 단순 이동 (좌표)
ros2 launch voice_robot_control joint_move.launch.py # ①-2 관절 이동 (각도)
ros2 launch voice_robot_control waypoint.launch.py   # ② 여러 지점
ros2 launch voice_robot_control gripper.launch.py    # ③ 그리퍼만
ros2 launch voice_robot_control pick_place.launch.py # ④ 집어서 옮기기
ros2 launch voice_robot_control gear.launch.py       # ⑤ 기어 조립
ros2 launch voice_robot_control stt.launch.py        # ⑥ 음성 인식
ros2 launch voice_robot_control nlp.launch.py        # ⑦ 말 → 명령
ros2 launch voice_robot_control voice_robot.launch.py # ⑧ 명령 → 로봇 동작
ros2 launch voice_robot_control voice.launch.py      # ⑥⑦⑧ 한 번에
ros2 launch voice_robot_control order.launch.py      # ⑨ 말로 시키는 로봇 (팀 프로젝트)
ros2 launch voice_robot_control viewer.launch.py     # 손끝 위치 보기
```

①~⑤ 는 한 번에 **하나만** 켜고, 할 일을 다 하면 스스로 끝납니다.
⑥⑦⑧ 은 계속 켜 두는 노드라 **셋을 각각 다른 터미널에서 동시에** 켭니다. Ctrl+C 로 끕니다.
⑨ 는 혼자서 다 하므로 **터미널 하나**면 됩니다.

### 말로 로봇 움직이기 (터미널 2개)

```bash
# 1  로봇 켜기 (위와 동일)
# 2  ros2 launch voice_robot_control voice.launch.py    ← 6,7,8 을 한 번에
```

마이크 없이 타이핑으로 시험할 때는 6 을 빼고 켭니다.

```bash
ros2 launch voice_robot_control voice.launch.py use_stt:=false
# 다른 터미널에서
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 2번 집어'"
```

셋을 **따로** 켜고 싶으면 (어디가 문제인지 찾을 때는 이쪽이 낫습니다)

```bash
ros2 launch voice_robot_control voice_robot.launch.py   # 8
ros2 launch voice_robot_control nlp.launch.py           # 7
ros2 launch voice_robot_control stt.launch.py           # 6
```

말할 것: "홈으로" / "기어 2번 집어" / "전체 조립해" / "앞으로" / "정지"

---

## 3. 실습 — 코드 고쳐 보기

각 파일 맨 위에 `▼▼▼ 여기를 바꿔가며 연습하세요 ▼▼▼` 구역이 있습니다.
**그 아래 숫자만 고치면 됩니다.** 나머지는 안 건드려도 됩니다.

### ① `move_node.py`

```python
TARGET = {"x": 0.500, "y": 0.000, "z": 0.500, "degree": 0.0}
                                                 # 어디로 갈까? [m] / 손목 [도]
SPEED = 0.15                                     # 얼마나 빠르게? (작을수록 느림)
```

| 값 | 뜻 | 넣을 수 있는 범위 |
|---|---|---|
| `x` | 앞으로 나간 거리 [m] | 0.30 ~ 0.60 |
| `y` | 왼쪽(+) / 오른쪽(-) [m] | -0.30 ~ 0.30 |
| `z` | 높이 [m] | 0.27 ~ 0.60 |
| `degree` | 손목을 돌리는 각도 [도] | -180 ~ 180 |

`degree` 는 **집게를 제자리에서 돌리는 각도**입니다. 손끝은 늘 바닥을 봅니다.
`0` 이면 지금까지와 똑같고, `90` 이면 집게를 90도 돌린 채로 갑니다.
물건이 비스듬히 놓여 있을 때 씁니다. 안 적으면 `0` 으로 봅니다.
**이 `degree` 는 좌표를 쓰는 모든 노드(①②④⑤⑧⑨)에서 똑같이 통합니다.**

**파일을 안 고치고 터미널에서 바로 찍을 수도 있습니다.**

```bash
ros2 launch voice_robot_control move.launch.py x:=0.45 y:=0.10 z:=0.40
ros2 launch voice_robot_control move.launch.py z:=0.60              # z 만 바꾸기
ros2 launch voice_robot_control move.launch.py degree:=90           # 손목만 돌리기
ros2 launch voice_robot_control move.launch.py x:=0.45 speed:=0.05  # 천천히
```

안 적은 값은 위의 `TARGET` / `SPEED` 를 그대로 씁니다. 여러 좌표를 빠르게
시험해 볼 때 편합니다. 마음에 드는 값을 찾으면 `TARGET` 에 옮겨 적으세요.

### ①-2 `joint_move_node.py`

좌표 대신 **관절 각도(도)** 를 직접 정합니다. 팔이 항상 같은 모양으로 움직입니다.

```python
TARGET = {                          # 어떤 자세로 갈까? 단위는 도(°)
    "joint_1": 30.0, "joint_2": 20.0, "joint_3": 70.0,
    "joint_4":  0.0, "joint_5": 90.0, "joint_6":  0.0,
}                                   # 안 적은 관절은 홈 값이 됩니다
SPEED = 0.15                        # 얼마나 빠르게?
```

관절이 돌 수 있는 범위 [도] — 벗어나면 경고가 뜨고 계획이 실패합니다.

| 관절 | 범위 | 관절 | 범위 |
|---|---|---|---|
| `joint_1` | -180 ~ 180 | `joint_4` | -180 ~ 180 |
| `joint_2` | -74 ~ 74 | `joint_5` | -114 ~ 114 |
| `joint_3` | -114 ~ 114 | `joint_6` | -180 ~ 180 |

> 지금 관절이 몇 도인지는 `ros2 topic echo /joint_states` 로 봅니다.
> 라디안으로 나오니 도로 바꾸려면 ×57.3 하세요.

### ② `waypoint_node.py`

```python
WAYPOINTS = [                       # 지나갈 지점들. 줄을 더 넣으면 늘어납니다
    {"x": 0.493, "y":  0.010, "z": 0.417},
    {"x": 0.493, "y": -0.218, "z": 0.417, "degree": 45},   # 지나가며 손목도 돌림
]
REPEAT = 1                          # 몇 바퀴?
RETURN_HOME = True                  # 끝나고 홈으로?
```

### ③ `gripper_node.py`

```python
USE_GRIPPER = True    # True 면 진짜 움직임. False 면 시늉만
OPEN_WIDTH = 500      # 열었을 때 50mm   ※ 단위가 1/10 mm 입니다
CLOSE_WIDTH = 150     # 닫았을 때 15mm
FORCE = 300           # 쥐는 힘 30N     ※ 단위가 1/10 N 입니다
MAX_RETRY = 3         # 못 잡으면 몇 번까지 다시?
```

> **단위 주의** — 그리퍼만 `1/10 mm`, `1/10 N` 을 씁니다.
> `500` 은 50mm, `300` 은 30N 입니다. EGP-C 40 은 열기/닫기 두 가지뿐이라
> 폭이 `325` 이상이면 열고 미만이면 닫습니다. 힘은 지정할 수 없어 무시됩니다.

> ③④⑤ 세 노드 모두 **닫힘 15mm / 힘 30N** 으로 같습니다.
> ③에서 잡히는 값이 ④⑤에서도 그대로 통합니다.

### ④ `pick_place_node.py`

```python
PICK  = {"x": 0.427, "y":  0.148, "z": 0.280, "degree": 0}   # 어디서 집을까?
PLACE = {"x": 0.426, "y": -0.153, "z": 0.280, "degree": 0}   # 어디에 놓을까?
APPROACH_OFFSET = 0.05                            # 위에서 접근하는 높이 [m]
USE_GRIPPER = True
```

> 물건이 비스듬히 놓여 있으면 `degree` 를 바꿔 집게를 돌려서 집습니다.
> 집을 때와 놓을 때 각도를 다르게 줄 수도 있습니다.

### ⑤ `gear_assembly_node.py`

```python
GEAR_TASKS = [
    {"pick":  {"x": 0.393, "y":  0.094, "z": 0.280, "degree": 0},
     "place": {"x": 0.393, "y": -0.206, "z": 0.280, "degree": 0}},
    ...
]
APPROACH_OFFSET = 0.05     # 위에서 접근하는 높이 [m]
USE_WIGGLE = True          # 마지막 기어를 좌우로 흔들어 끼울까?
USE_GRIPPER = True         # 실제 그리퍼를 쓸까?
```

> 좌우로 흔드는 동작(`USE_WIGGLE`)도 `place` 의 `degree` 를 기준으로
> ±`WIGGLE_YAW_DEG` 만큼 돌립니다. 즉 `degree` 를 바꾸면 흔드는 기준도 같이 돕니다.

### ⑥ `stt_node.py`

```python
LANGUAGE = "ko-KR"        # 한국어. 영어로 하려면 "en-US"
ENERGY_THRESHOLD = 0      # 0 = 시작할 때 주변 소음에 맞춰 자동으로 정함
PAUSE_THRESHOLD = 0.8     # 이만큼 조용해지면 '말이 끝났다'
PHRASE_TIME_LIMIT = 5.0   # 한 번에 최대 몇 초까지 들을까
MIC_INDEX = -1            # -1 = 기본 마이크. 실행하면 목록이 나옵니다
```

> **감도(`ENERGY_THRESHOLD`) 가 이 실습의 핵심입니다.**
> 이 컴퓨터에서 자동 측정하면 **4000~5500** 쯤 나옵니다. 기본값 300 을 그대로 쓰면
> 주변 소음까지 전부 말로 알아들어 버립니다. 조용한 곳이면 낮게, 시끄러우면 높게.

마이크가 없거나 안 될 때는 말한 것처럼 흉내낼 수 있습니다.

```bash
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 1번 집어'"
ros2 topic echo /stt_result       # 잘 들리는지 확인만
```

### ⑦ `nlp_node.py` 와 `config/keyword_map.yaml`

무슨 말을 알아들을지는 **`config/keyword_map.yaml`** 에 적혀 있습니다.
낱말을 추가하고 노드를 껐다 켜면 끝입니다. **다시 빌드하지 않습니다.**

```yaml
commands:
  pickplace:
    - 조립
    - 옮겨
    - 우리팀만의말     # ← 이렇게 한 줄 추가
```

바뀌는 과정이 화면에 그대로 나옵니다.

```
[들은 말] "기어 2번 집어줘"
   ├ 명령어 찾기   "집어줘" → pick
   ├ 번호 찾기     "2번" → 2번
   └ 만들어진 명령  >>> pick 2
```

만들어지는 명령: `home` / `pick N` / `place N` / `pickplace N` /
`pickplace all` / `jog forward` / `open` / `close` / `stop`

> **명령이 방향보다 먼저입니다.** "위로 조립해" 처럼 둘 다 들어 있으면
> '위'(방향)가 아니라 '조립'(명령)으로 봅니다.
> 그리고 긴 낱말을 먼저 확인하므로 "왼쪽" 이 "왼" 에 가려지지 않습니다.

#### 우리 팀 명령어 추가하기 — 4단계

**1. 파일 열기**

```bash
gedit ~/doosan_voice_ws/src/voice_robot_control/config/keyword_map.yaml
```

**2. 하고 싶은 말을 한 줄 추가** (원하는 항목 밑에 `- 낱말`)

```yaml
commands:
  pickplace:
    - 조립
    - 출발        # ← 우리 팀 말
```

**3. 미리 보기** — 로봇도 ROS도 마이크도 필요 없습니다

```bash
python3 ~/doosan_voice_ws/src/voice_robot_control/voice_robot_control/nlp_node.py "기어 2번 출발"
```

```
[들은 말] "기어 2번 출발"
   · 명령어 찾기   "출발" → pickplace
   · 번호 찾기     "2번" → 2번
   → pickplace 2
```

**4. 반영** — `nlp_node` 를 Ctrl+C 로 끄고 다시 켜기.
`install` 이 `src` 로 이어져 있어 **다시 빌드하지 않습니다.**

> **낱말 만들 때 주의**
> 짧은 낱말은 다른 말 속에 숨어듭니다. 예를 들어 '전체'를 뜻하는 낱말로
> `다` 를 넣으면 `가져다놔` 안에도 `다` 가 있어서, 1번만 시켜도 4개를 다 해버립니다.
> 노드를 켤 때 이런 경우를 자동으로 잡아 알려줍니다.
>
> ```
> · "다" (전체) 가 "가져다놔" (명령/pickplace) 안에 들어 있습니다
>   → "가져다놔" 라고만 말해도 "다" 를 말한 것으로 봅니다
> ```
>
> 들여쓰기를 틀렸을 때도 몇째 줄인지 알려줍니다.

#### 우리 팀 **동작**까지 새로 만들기 — 파일 2개

낱말만 바꾸는 게 아니라 **없던 동작을 새로 만들 수도** 있습니다.
파이썬은 건드리지 않습니다.

**1. `config/my_actions.yaml` 에 동작을 만든다**

```yaml
actions:
  인사:
    - 홈
    - 이동: {x: 0.40, y:  0.15, z: 0.50}
    - 이동: {x: 0.40, y: -0.15, z: 0.50}
    - 홈
```

쓸 수 있는 한 줄

| 줄 | 하는 일 |
|---|---|
| `- 홈` | 홈 자세로 |
| `- 이동: {x: 0.4, y: 0.0, z: 0.45}` | 그 좌표로 [m] |
| `- 이동: {x: 0.4, y: 0.0, z: 0.45, degree: 90}` | 손목을 90도 돌린 채로 [도] |
| `- 관절: [0, 0, 90, 0, 90, 0]` | 관절 6개를 그 각도로 [도] |
| `- 집기: 2` / `- 놓기: 2` | 2번 기어 집기 / 놓기 |
| `- 열기` / `- 닫기` | 그리퍼 |
| `- 기다리기: 1.0` | 그만큼 쉬기 [초] |

**2. `config/keyword_map.yaml` 에 같은 이름으로 낱말을 붙인다**

```yaml
commands:
  인사:          # ← my_actions.yaml 의 이름과 똑같이
    - 인사해
    - 안녕
```

**3. `nlp_node` 와 `voice_robot_node` 를 껐다 켠다** (다시 빌드 안 함)

**4. "안녕" 이라고 말한다**

```
[들은 말] "안녕"  →  인사
우리 팀 동작 '인사' — 모두 4단계
  [1/4] 홈
  [2/4] 이동: {'x': 0.4, 'y': 0.15, 'z': 0.5}
  ...
>>> 인사 완료
```

예시로 `인사` / `집게운동` / `둘만옮기기` 세 개가 이미 만들어져 있습니다.
그대로 말해 보고, 숫자를 고쳐가며 우리 팀 동작을 만들면 됩니다.

> 좌표를 모르겠으면 위치 보기를 켜서 확인하세요.
> `ros2 launch voice_robot_control viewer.launch.py`

#### ⑥ 없이 ⑦ 만 써도 됩니다

⑦ 은 `/stt_result` 토픽을 구독할 뿐이라, 거기에 글자를 넣어주는 게
마이크든 손타이핑이든 상관없습니다.

```bash
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 2번 집어'"
```

| 켜는 것 | 할 수 있는 일 |
|---|---|
| ⑦ 만 | 글자가 어떤 명령이 되는지 확인 (로봇·마이크 불필요) |
| ⑦ + ⑧ | 타이핑으로 로봇을 실제로 움직임 (마이크 불필요) |
| ⑥ + ⑦ + ⑧ | 말로 로봇을 움직임 |
| ⑧ 만 | `/robot_command` 에 직접 넣어 움직임 |

#### 어디가 문제인지 찾기

```
말했는데 로봇이 안 움직인다
  → /stt_result 에 직접 넣어본다
      움직이면   → ⑥ 문제 (감도 / 마이크 번호 / 인터넷)
      안 움직이면 → ⑦ 또는 ⑧ 문제
```

```bash
ros2 topic echo /stt_result      # ⑥ 이 뭘 알아들었나
ros2 topic echo /robot_command   # ⑦ 이 뭘 만들었나
ros2 topic echo /robot_status    # ⑧ 이 뭘 했나 (스피커로도 같은 말이 나옵니다)
```

### ⑧ `voice_robot_node.py`

```python
GEAR_TASKS = [ ... ]       # 기어 좌표 (⑤ 와 같은 값)
APPROACH_OFFSET = 0.05     # 위에서 접근하는 높이 [m]
JOG_STEP = 0.05            # "앞으로" 한 번에 갈 거리 [m]
USE_TTS = True             # 한 일을 스피커로 읽어 줄까?
TTS_LANG = "ko"            # 영어로 하려면 "en"
USE_GRIPPER = True
SPEED = 0.15
```

**한 일을 소리로 알려 줍니다.** `pickplace 2` 를 끝내면 스피커가
"pickplace 2 완료" 라고 읽어 줍니다. 같은 말이 `/robot_status` 토픽으로도
나가니, 소리를 끄고(`USE_TTS = False`) 토픽만 쓸 수도 있습니다.

> `gtts` / `pygame` 이 없으면 알려주고 **글자로만** 보여줍니다. 실습은 그대로
> 진행됩니다. 소리를 내려면 `pip install gtts pygame` 하세요.
> 첫 문장은 구글 서버에서 받아오느라 1~2초 걸리고, 같은 문장은 저장해 두고
> 다시 씁니다.

> **로봇은 말이 끝날 때까지 기다리지 않습니다.** 말하기는 따로 스레드를
> 두고 처리합니다. 안 그러면 "전체 조립" 처럼 기어마다 한 마디씩 하는 경우
> 소리를 받아오는 시간이 다 쌓여서 팔이 멈춰 서 있게 됩니다.
> `정지` 라고 하면 기다리던 명령과 함께 **하려던 말도 같이 버립니다.**

알아듣는 명령 (⑦ 이 만들어 보내는 것과 같습니다)

| 명령 | 뜻 |
|---|---|
| `home` | 홈 자세로 |
| `pick 2` / `place 2` | 2번 기어 집기 / 놓기 |
| `pickplace 2` | 2번 기어 집어서 옮기기 |
| `pickplace all` | 전체 기어 순서대로 |
| `jog forward` | 앞으로 조금 (backward/left/right/up/down) |
| `open` / `close` | 그리퍼 열기 / 닫기 |
| `stop` | 기다리는 명령 모두 취소 |

말 없이 직접 시켜볼 수도 있습니다.

```bash
ros2 topic pub --once /robot_command std_msgs/msg/String "data: 'pick 2'"
```

> **`stop` 은 기다리던 명령만 취소합니다.** 이미 시작한 동작은 끝까지 갑니다.
> 급할 때는 **비상정지 버튼**을 누르세요.

### ⑨ `order_node.py` — 팀 프로젝트

**말로 물어보고, 듣고, 시킨 작업을 하는 로봇**입니다.

```bash
# 터미널 1 — 로봇
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609
# 터미널 2 — 말로 시키기
ros2 launch voice_robot_control order.launch.py
```

```
🔊 로봇 : "어떤 동작을 원하시나요? 콜라, 사이다 중에 골라 주세요."
🎤 사람 : "콜라"
      → if 문이 '콜라' 를 골라내고 → 작업1_하기() 가 실행됩니다
🔊 로봇 : "콜라 끝났습니다."
      → 다시 처음으로

끝낼 때는 "그만" 이라고 말하거나 Ctrl+C
```

**맨 위에서 고칠 것은 작업 이름, 좌표, 집게 폭입니다.**

```python
# 1. 무슨 말을 들으면 어떤 작업을 할까요?
#    "콜라", "1번", "왼쪽" 처럼 아무 말이나 넣어도 됩니다
작업1_이름 = "콜라"
작업2_이름 = "사이다"

# 2. 좌표 [m] + 손목 각도 [도]
#    작업 1 에서 쓰는 자리
좌표1 = {"x": 0.393, "y":  0.094, "z": 0.330, "degree": 0}    # 집을 물건 위
좌표2 = {"x": 0.393, "y":  0.094, "z": 0.280, "degree": 0}    # 집을 물건
좌표3 = {"x": 0.393, "y": -0.206, "z": 0.330, "degree": 0}    # 놓을 자리 위
좌표4 = {"x": 0.393, "y": -0.206, "z": 0.280, "degree": 0}    # 놓을 자리

#    작업 2 에서 쓰는 자리
좌표5 = {"x": 0.392, "y":  0.200, "z": 0.330, "degree": 0}
좌표6 = {"x": 0.392, "y":  0.200, "z": 0.280, "degree": 0}
좌표7 = {"x": 0.392, "y": -0.101, "z": 0.330, "degree": 0}
좌표8 = {"x": 0.392, "y": -0.101, "z": 0.280, "degree": 0}

# 3. 집게를 얼마나 벌리고 얼마나 세게 쥘까?  (단위가 1/10 입니다)
놓을_폭 = 500        # 50mm — 놓을 때 벌리는 폭
잡을_폭 = 150        # 15mm — 잡을 때 좁히는 폭
잡는_힘 = 300        # 30N  — 약한 물건이면 100(10N) 정도로
```

> **왜 자리를 두 개씩 적나요?** 물건 바로 위(`좌표1`)를 한 번 들렀다가
> 내려가야(`좌표2`) 옆에서 부딪히지 않습니다. `x`, `y` 는 같고 `z` 만 다릅니다.

> **`잡을_폭` 은 물건보다 좁아야 합니다.** 물건보다 넓으면 집게가 겉만
> 스치고 지나가서 `물건을 못 잡은 것 같습니다` 가 뜹니다.

> **`degree` 는 집게를 돌리는 각도입니다** (-180 ~ 180). 물건이 비스듬히
> 놓여 있으면 그만큼 돌려서 잡습니다. 잘 모르겠으면 `0` 으로 두세요.
> 집을 자리와 그 위 자리는 보통 같은 `degree` 를 씁니다.

**작업마다 자기 블록이 따로 있습니다.** 작업 1 과 작업 2 를 아주 다른 동작으로
만들어도 됩니다. 줄을 지우거나, 순서를 바꾸거나, 새 줄을 끼워 넣으면 그대로 바뀝니다.
`time.sleep` 의 숫자도 직접 고칩니다 (로봇이 흔들리면 늘리고, 느리면 줄이세요).

```python
def 작업1_하기():
    말하기(f"{작업1_이름} 시작하겠습니다. 잠시만 기다려 주세요.")

    그리퍼(놓을_폭)         # 집게를 열고
    time.sleep(1.0)

    이동(좌표1)            # 물건 위로 가서
    time.sleep(1.0)

    이동(좌표2)            # 내려가서
    time.sleep(1.0)

    그리퍼(잡을_폭, 잡는_힘)  # 잡고
    time.sleep(1.0)

    이동(좌표1)            # 들어 올리고
    time.sleep(1.0)

    이동(좌표3)            # 놓을 자리 위로 옮기고
    time.sleep(1.0)

    이동(좌표4)            # 내려가서
    time.sleep(1.0)

    그리퍼(놓을_폭)         # 놓고
    time.sleep(1.0)

    이동(좌표3)            # 빠져나오고
    time.sleep(1.0)

    홈으로()               # 제자리로

    말하기(f"{작업1_이름} 끝났습니다.")


def 작업2_하기():
    ...                    # 좌표5~8 을 쓰는 블록. 완전히 다른 동작이어도 됩니다
```

고른 작업을 부르는 곳은 이 `if` 문 하나입니다.

```python
if 그만_할_때 in 손님_말:
    말하기(끝인사)
    return False

elif 작업1_이름 in 손님_말:
    작업1_하기()          # ← 작업 1 의 블록이 실행됩니다

elif 작업2_이름 in 손님_말:
    작업2_하기()          # ← 작업 2 의 블록이 실행됩니다

else:
    말하기(못_알아들었을_때)
```

쓸 수 있는 블록은 이게 전부입니다 (`blocks.py` 에 들어 있습니다).

| 블록 | 하는 일 |
|---|---|
| `말하기("안녕하세요")` | 스피커로 말한다 (`print` 처럼 여러 개를 줘도 됩니다) |
| `듣기()` | 마이크로 듣고 글자로 돌려준다 |
| `홈으로()` | 홈 자세로 간다 |
| `이동(좌표1)` | 그 좌표로 간다 (`degree` 가 있으면 손목도 돌린다) |
| `이동(좌표1, 위로=0.05)` | 그 자리의 5cm 위로 간다 |
| `그리퍼(500)` | 집게를 50mm 로 벌린다 (놓기) |
| `그리퍼(150)` | 집게를 15mm 로 좁힌다 (잡기). 못 잡으면 `False` |
| `그리퍼(150, 100)` | 좁히되 약하게 (10N) |
| `time.sleep(1.0)` | 1초 쉰다 |

**로봇·마이크 없이 자리에서 연습하기** — 파일 맨 위의 스위치를 내리세요.

```python
로봇_사용 = False      # 팔을 안 움직이고 화면에만 찍습니다
마이크_사용 = False    # 마이크 대신 /stt_result 를 기다립니다
```

```bash
ros2 run voice_robot_control order_node
# 다른 터미널에서 말 대신 시키기
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '콜라'"
```

> **작업을 늘리려면** 세 가지를 하면 됩니다.
> 1. `작업3_이름` 과 쓸 좌표(`좌표9`, `좌표10` ...)를 맨 위에 추가
> 2. `def 작업3_하기():` 를 만들고 블록을 쌓기 (작업2 를 복사해서 고치면 쉽습니다)
> 3. `if` 문에 `elif 작업3_이름 in 손님_말: 작업3_하기()` 한 덩이 추가

---

## 4. 좌표를 모를 때

터미널 3에서 위치 보기를 켜면 손끝이 지금 어디 있는지 계속 알려줍니다.

```bash
ros2 launch voice_robot_control viewer.launch.py
```

```
손끝 위치   x=+0.398  y=+0.096  z=+0.280  [m]
  손끝 방향   +180 / +0 / +180  [도]
  손목 각도   degree=+0  [도]
  관절 각도   J1=+12  J2=+30  ...
```

이 숫자를 그대로 `TARGET` 이나 `GEAR_TASKS` 에 옮겨 적으면 됩니다.
`손목 각도` 의 값이 그대로 `degree` 에 적을 값입니다.
이름을 붙여 파일로 남기고 싶으면:

```bash
ros2 topic pub --once /record_pose std_msgs/msg/String "data: '내자리'"
# → ~/내가_저장한_자세.yaml 에 쌓입니다
#   내자리: {x: 0.398, y: 0.096, z: 0.280, degree: 0}   ← 복사해 쓰면 됩니다
```

---

## 5. 안전장치

**좌표**가 아래 범위를 벗어나면 **경계값으로 잘라내고 경고**를 띄웁니다.
`robot_common.py` 의 `SAFE_*` 에 있습니다.

| 축 | 잘리는 범위 [m] | 실습 권장 [m] |
|----|----------|----------|
| x | 0.0 이상 | 0.30 ~ 0.60 |
| y | -0.3 ~ 0.3 | -0.30 ~ 0.30 |
| z | 0.27 이상 | 0.27 ~ 0.60 |

**각도**는 잘라내지 않습니다. 잘라 버리면 학생이 모르는 사이에 엉뚱한 자세로
가기 때문입니다. 대신 이렇게 합니다.

| 값 | 범위 | 벗어나면 |
|---|---|---|
| `degree` (손목) | -180 ~ 180 | 같은 방향인 값으로 접습니다 (270 → -90) |
| `joint_1` ~ `joint_6` | `robot_common.py` 의 `JOINT_RANGE_DEG` | 경고가 뜨고 계획이 실패합니다 |

범위 안이어도 팔이 닿지 않으면(팔 길이 0.9m) `가는 길을 찾지 못했습니다` 가 뜹니다.

처음 해 볼 때는 각 파일의 `SPEED` 를 `0.05` 정도로 낮추고 비상정지에 손을 올려 두세요.

---

## 6. 잘 안 될 때

| 화면에 뜨는 말 | 무엇을 확인할까 |
|---|---|
| `libgeometric_shapes.so.2.3.2: cannot open...` | `ws_moveit` 을 source 했습니다. 새 터미널을 여세요 |
| `Unable to configure planning scene monitor` | 터미널 1(로봇)이 안 떠 있습니다 |
| `가는 길을 찾지 못했습니다` | 로봇 팔이 닿지 않는 좌표입니다. 위치 보기로 확인 후 가까운 값부터 |
| `길은 찾았는데 로봇이 움직이지 않았습니다` | 계획은 됐지만 로봇이 거부했습니다. 비상정지 / 연결 확인 |
| `그리퍼에 연결하지 못했습니다` | `USE_GRIPPER = False` 로 두면 시늉만 하고 계속 진행합니다 |
| `마이크를 열지 못했습니다` | 화면의 마이크 목록에서 번호를 골라 `MIC_INDEX` 에 적으세요 |
| `구글 음성 인식 서버에 연결하지 못했습니다` | 인터넷 연결을 확인하세요 (⑥ 은 인터넷이 필요합니다) |
| 소음까지 계속 알아들음 | `ENERGY_THRESHOLD` 를 올리세요 (이 컴퓨터는 4000~5500 정도) |
| `아는 낱말이 하나도 없습니다` | `keyword_map.yaml` 에 그 말을 추가하고 ⑦ 을 다시 켜세요 |
| `소리를 낼 수 없어 글자로만 보여줍니다` | ⑧⑨ 의 TTS 입니다. `pip install gtts pygame` 후 다시 켜세요 |
| ⑨ 가 `/stt_result 를 기다립니다` | 마이크를 못 열었습니다. 다른 터미널에서 `ros2 topic pub` 로 시키거나 마이크를 확인하세요 |
| `몇 번인지 말해주세요` | "기어 2번 집어" 처럼 번호를 넣거나 "전체" 라고 하세요 |

---

## 7. 파일 구성

```
voice_robot_control/
├── legacy/                 #5 실습 노드 (현재 강의에서는 사용하지 않음)
│   ├── move_node.py            ① 단순 이동 (좌표)
│   ├── joint_move_node.py      ①-2 관절 이동 (각도)
│   ├── waypoint_node.py        ② 여러 지점
│   ├── gripper_node.py         ③ 그리퍼
│   ├── pick_place_node.py      ④ 집어서 옮기기
│   └── gear_assembly_node.py   ⑤ 기어 조립
├── stt_node.py             ⑥ 음성 인식
├── nlp_node.py             ⑦ 말 → 로봇 명령
├── voice_robot_node.py     ⑧ 명령 → 로봇 동작
├── order_node.py           ⑨ 말로 시키는 로봇 (팀 프로젝트, 학생이 고치는 파일)
├── position_viewer.py      손끝 위치 보기
├── launch_helper.py        launch 파일 공용 부분
│
└── core/                   학생이 고치지 않는 공용 부품
    ├── robot_common.py         여러 노드가 함께 쓰는 부분
    │                           (홈 자세, 안전 영역, 경로 계획+실행)
    ├── gripper_control.py      그리퍼 연결 (실물 없으면 시늉)
    ├── egp_c40.py              EGP-C 40 그리퍼 드라이버
    ├── speaker.py              말하기 (TTS) — ⑧ 과 ⑨ 가 같이 씁니다
    ├── pose_utils.py           각도 ↔ 쿼터니언 변환
    └── blocks.py               ⑨ 가 쓰는 블록 (말하기/듣기/이동/그리퍼)

학생이 고치는 설정 파일은 config/ 에 있습니다.
    config/keyword_map.yaml   어떤 말을 어떤 명령으로 볼지
    config/my_actions.yaml    우리 팀이 만드는 새 동작
```

로봇을 움직이는 방법 자체는 **`robot_common.py` 의 `plan_and_execute()` 한 곳**에
모여 있습니다. 모든 노드가 이 함수를 씁니다.
