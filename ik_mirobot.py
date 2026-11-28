"""
Step 2-2: Mirobot IK (tool이 수직 아래를 향하는 pick & place 자세)

Target: tool 끝 position (x, y, z) + yaw (tool을 z축으로 얼마나 돌릴지)
조건:  tool은 항상 수직 아래를 향한다 (roll = pitch = 0)

핵심 idea (decoupling, 문제 쪼개기):
  - J1, J2, J3 → tool을 "어디에" 둘지 (position)
  - J5, J6     → tool을 "어느 방향으로" 향할지 (orientation)
  - J4         → 0으로 고정 (수직 아래 자세에서는 필요 없음)

풀이 순서:
  1. J1: 위에서 내려다본 target 방향
  2. Wrist(J5) 위치: tool이 수직 아래니까, target에서 tool 길이만큼 위
  3. J2, J3: 옆모습에서 J2 → wrist까지 2-link IK
  4. J5: pitch가 0이 되도록 J2 + J3 회전을 상쇄
  5. J6: 원하는 yaw가 되도록 J1 회전을 보정
"""
import numpy as np

from fk_6axis import fk_6axis, matrix_to_rpy

# Mirobot geometry (fk_6axis.py와 같음)
J2_X, J2_Z = 29.69, 127.0       # base origin → J2
UPPER_ARM = 108.0               # J2 → J3 (home에서 수직 위)
FORE_X, FORE_Z = 168.98, 20.0   # J3 → J5 wrist (home에서 앞으로, 살짝 위)
TOOL = 24.27                    # J5 → tool 끝 (아래로)

# 2-link로 보기 위한 값
L1 = UPPER_ARM
L2 = np.hypot(FORE_X, FORE_Z)                  # J3 → wrist 직선 거리 = 170.16
FORE_OFFSET = np.degrees(np.arctan2(FORE_Z, FORE_X))  # forearm이 살짝 위로 꺾인 angle = 6.75°


def ik_mirobot(x, y, z, yaw=0.0):
    """
    Tool 끝 target (x, y, z) mm, yaw deg → joint angles (j1..j6) deg.
    닿을 수 없으면 ValueError.
    """
    # 1. J1: 위에서 본 target 방향 (atan2로 360° 전체 구분)
    j1 = np.degrees(np.arctan2(y, x))

    # 2. Wrist 위치: tool이 수직 아래이므로 tool 길이만큼 위
    wz = z + TOOL

    # 3. 옆모습 (r, h) 평면으로 옮기기. 기준점은 J2.
    #    r: base 축에서 수평 거리,  h: 높이
    r = np.hypot(x, y) - J2_X
    h = wz - J2_Z

    #    2-link IK (ik_2link.py와 같은 공식).
    #    여기서 angle은 "수평(r축)에서 반시계 +" 기준인 수학 표준 angle.
    cos_q2 = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if cos_q2 < -1 or cos_q2 > 1:
        raise ValueError(f"target ({x}, {y}, {z})은 reach 밖")
    q2 = -np.arccos(cos_q2)   # Mirobot의 평소 자세 = 팔꿈치가 위로 솟은 쪽 (−)
    q1 = np.arctan2(h, r) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q1, q2 = np.degrees(q1), np.degrees(q2)

    #    수학 표준 angle (q1, q2) → Mirobot joint angle (j2, j3) 변환
    #    - Home에서 upper arm은 수직 위 → q1 = 90°  일 때 j2 = 0
    #    - Home에서 forearm은 upper arm보다 (90° − 6.75°) 시계 방향 → q2 = −83.25° 일 때 j3 = 0
    #    - Mirobot의 +는 옆모습에서 시계 방향 = 수학 표준의 − 방향
    j2 = 90.0 - q1
    j3 = -(q2 - (FORE_OFFSET - 90.0))

    # 4. J5: J2, J3로 기울어진 만큼 반대로 꺾어서 tool을 수직 아래로
    j5 = -(j2 + j3)

    # 5. J4 = 0,  J6: J1으로 돌아간 만큼 빼서 원하는 yaw 맞추기
    j4 = 0.0
    j6 = yaw - j1

    return np.array([j1, j2, j3, j4, j5, j6])


if __name__ == "__main__":
    # FK(IK(target)) == target self-check
    targets = [
        (198.67, 0, 230.72, 0),     # home
        (200, 0, 150, 0),
        (180, 80, 100, 0),
        (150, -100, 60, 30),
        (220, 50, 20, -45),         # table 근처 (pick 높이)
    ]
    for x, y, z, yaw in targets:
        joints = ik_mirobot(x, y, z, yaw)
        T = fk_6axis(*joints)
        pos = T[:3, 3]
        rpy = matrix_to_rpy(T[:3, :3])
        pos_err = np.linalg.norm(pos - [x, y, z])
        print(f"target=({x}, {y}, {z}, yaw={yaw})")
        print(f"  IK joints = {np.round(joints, 2)}")
        print(f"  FK(IK)    = pos {np.round(pos, 3)}  rpy {np.round(rpy, 3)}  pos error={pos_err:.6f} mm")

    try:
        ik_mirobot(500, 0, 100)
    except ValueError as e:
        print(f"\nunreachable check ✅ {e}")
