# Mirobot Pick & Place

Camera로 table 위의 block을 찾고, WLKATA Mirobot (6-axis desktop robot arm)이 그 block을 집어 옮기는 project.
SDK를 호출만 하는 데서 끝나지 않고, robot이 내부에서 하는 계산(kinematics, coordinate transform)을 직접 implement하면서 이해하는 것이 목표.

> Demo video / GIF: _coming soon_

## System architecture

```
Python script ─→ SDK (wlkata-mirobot-python) ─→ G-code ─→ USB serial (CH340) ─→ Mirobot firmware ─→ Motor
                                                                                        │
Python script ←──────────── Pose / joint angle status ←────────────────────────────────┘
```

PC는 "어디로 가"라는 command만 보내고, IK와 motor control 같은 무거운 computation은 robot controller가 담당한다.

## Roadmap

### Step 0. Environment & Connection ✅
**"PC는 robot과 어떻게 대화하는가"**
- **Why**: 모든 작업의 출발점. Command가 어떤 경로로 robot까지 가는지 알아야 문제가 생겼을 때 어디를 볼지 안다.
- **What**: Python virtual environment, USB serial connection, G-code, homing, 첫 coordinate move
- **Output**: `test_connect.py`, `find_port.py` (USB VID로 robot port 자동 탐색)

### Step 1. Forward Kinematics (Python) 🔄
**"Robot은 자기 손이 어디 있는지 어떻게 아는가"**
- **Why**: Joint angle만으로 end-effector position을 계산하는 원리를 이해한다. Rotation matrix와 homogeneous transformation은 robotics의 기본 linear algebra이고, 이후 모든 coordinate transform의 바탕이 된다.
- **What**:
  1. Ground truth data collection: joint angle ↔ robot이 report한 pose
  2. 2-link planar FK로 concept 잡기
  3. Mirobot 6-axis FK를 numpy로 implement (DH parameter)
  4. Robot report 값과 error 비교
- **Output**: `collect_data.py`, `data/fk_samples.csv`, FK module, error analysis

### Step 2. Inverse Kinematics (Python)
**"손을 원하는 곳에 두려면 joint를 어떻게 움직여야 하는가"**
- **Why**: FK의 반대 방향. Motion planning은 "target pose에 가기 위한 joint angle과, 여러 solution 중 어떤 것을 고를지"를 푸는 일이라 IK가 기본이 된다. Solution이 여러 개이거나 없을 수 있다는 점이 FK와의 핵심 차이.
- **What**:
  1. 2-link analytic IK (law of cosines + `atan2`), elbow-up / elbow-down 두 solution 확인
  2. 직접 만든 FK를 이용한 numerical IK (Jacobian 기초)
  3. `FK(IK(target)) == target` self-check
  4. 내 IK로 구한 angle을 `set_joint_angle()`로 보내고, firmware IK(`set_tool_pose()`) 결과와 비교
- **Output**: IK module, solution 비교 결과

### Step 3. Forward Kinematics (C++ / Eigen)
**"같은 수학을 robot software의 실제 언어로"**
- **Why**: Industrial robot software는 performance 때문에 C++로 작성되는 경우가 많다. Python으로 검증한 algorithm을 C++로 옮기며 두 language의 결과가 일치하는지 확인한다.
- **What**: Eigen library로 FK 재구현, Python version과 같은 test data로 검증
- **Output**: C++ FK module, Python vs C++ 결과 비교

### Step 4. Vision
**"Camera는 block을 어떻게 찾는가"**
- **Why**: Robot이 무엇을 집을지 스스로 판단하려면 먼저 "보는" 능력이 필요하다.
- **What**: OpenCV로 top-down webcam image에서 color 기반 block detection, pixel coordinate 추출
- **Output**: Block detection module, detection 결과 image

### Step 5. Camera-Robot Calibration
**"Camera가 본 위치를 robot은 어떻게 이해하는가"**
- **Why**: Camera는 pixel 단위로, robot은 base frame 기준 mm 단위로 세상을 본다. 두 coordinate system을 연결해야 "본 것"을 "집을" 수 있다.
- **What**: Robot tool로 table 위 4점을 찍어 pixel ↔ robot coordinate homography 계산 (table plane 가정)
- **Output**: Calibration module, 변환 error 측정 결과

### Step 6. Pick & Place Integration
**"보고, 계산하고, 집는다"**
- **Why**: 지금까지의 module을 하나의 pipeline으로 연결해서 실제로 동작하는 system을 만든다.
- **What**: Detection → coordinate transform → approach → pick → place → home
- **Output**: Demo video, 성공률 측정, limitation과 next step 정리

## Project structure

```
mirobot-pick-place/
├── README.md
├── requirements.txt
├── find_port.py        # USB VID로 Mirobot serial port 탐색
├── test_connect.py     # Connection & 첫 move test
├── collect_data.py     # FK ground truth data collection
├── data/               # 수집한 data
└── docs/
    ├── glossary.md     # 용어집
    └── til/            # Daily learning log (TIL)
```

## Setup

```bash
# Python 3.9 사용 (SDK가 Python 3.11+와 호환되지 않음, docs/til/2026-11-24.md 참고)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python test_connect.py
```

## Learning notes

- [Glossary](docs/glossary.md)
- [TIL (Today I Learned)](docs/til/)
