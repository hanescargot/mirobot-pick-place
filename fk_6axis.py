"""
Step 1-3: 6-axis FK with 4x4 homogeneous transformation

fk_planar.py에서 한 "화살표를 누적 회전해서 더하기"를 3D로 확장한다.
회전(R)과 이동(p)을 하나의 4x4 matrix T로 묶으면,
"돌리고 → 이동하고 → 돌리고 → 이동하고..."가 전부 matrix 곱하기가 된다.

    T_tool = Rz(j1) · Trans(J2_POS) · Ry(j2) · Trans(UPPER_ARM) · Ry(j3) · ...

T_tool의 오른쪽 위 3x1 = tool 끝 position, 왼쪽 위 3x3 = tool orientation.
"""
import csv

import numpy as np


# ---------- 3D rotation matrices ----------
# 2D에서 배운 규칙 그대로: "각 column = 기본 화살표가 도착한 곳"
# 회전축 방향의 기본 화살표는 움직이지 않는다.

def rot_y(theta_deg):
    """y축 기준 회전. z → x 방향으로 돌아감 (Mirobot J2, J3, J5의 + 방향 = 팔이 앞으로 숙여짐)."""
    t = np.radians(theta_deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def rot_z(theta_deg):
    """
    z축 기준 회전. x → y 방향으로 돌아감 (Mirobot J1, J6).
    TODO: 구현하세요.
    Hint: z축 화살표 (0,0,1)은 그대로. x, y 부분은 2D 반시계 방향 matrix와 똑같음.
    """
    t = np.radians(theta_deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[ c, -s, 0],
                     [ s, c, 0],
                     [0, 0, 1]])


    raise NotImplementedError("rot_z를 구현하세요")


def rot_x(theta_deg):
    """
    x축 기준 회전. y → z 방향으로 돌아감 (Mirobot J4, forearm roll).
    TODO: 구현하세요.
    Hint: x축 화살표 (1,0,0)은 그대로. y, z 부분은 2D 반시계 방향 matrix와 똑같음.
    """

    t = np.radians(theta_deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[ 1, 0, 0],
                     [ 0, c, -s],
                     [0, s, c]])

    raise NotImplementedError("rot_x를 구현하세요")
# ---------- 4x4 homogeneous transformation ----------

def rot(R):
    """회전만 하는 4x4 matrix:  [[R, 0], [0, 1]]"""
    T = np.eye(4)
    T[:3, :3] = R
    return T


def trans(x, y, z):
    """이동만 하는 4x4 matrix:  [[I, p], [0, 1]]"""
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


# ---------- Mirobot FK ----------

def fk_6axis(j1, j2, j3, j4, j5, j6):
    """
    6개 joint angle(deg) → tool의 4x4 transformation matrix.

    Chain (base → tool), link length는 fk_planar.py와 같음:
      1. J1: base에서 z축 회전
      2. J2_POS만큼 이동 (29.69, 0, 127)
      3. J2: y축 회전
      4. UPPER_ARM만큼 이동 (0, 0, 108)
      5. J3: y축 회전
      6. 위로 20 이동 (0, 0, 20)
      7. J4: x축 회전 (forearm 길이 방향 축)
      8. 앞으로 168.98 이동 (168.98, 0, 0)
      9. J5: y축 회전
     10. J6: z축 회전 (tool 축)
     11. TOOL만큼 이동 (0, 0, -24.27)

    TODO: 6번부터 11번까지 채우세요. 1~5번은 예시로 써 두었어요.
    """
    T = (rot(rot_z(j1))
         @ trans(29.69, 0, 127)
         @ rot(rot_y(j2))
         @ trans(0, 0, 108)
         @ rot(rot_y(j3))
         # TODO: 6 ~ 11
         @ trans(0, 0, 20)          # 6. 위로 20
         @ rot(rot_x(j4))           # 7. J4: forearm 축으로 비틀기
         @ trans(168.98, 0, 0)      # 8. 앞으로 168.98
         @ rot(rot_y(j5))           # 9. J5: 손목 꺾기
         @ rot(rot_z(j6))           # 10. J6: tool 축으로 돌리기
         @ trans(0, 0, -24.27)      # 11. 아래로 24.27
         )
    return T


# ---------- Orientation 비교용 (그대로 사용) ----------

def matrix_to_rpy(R):
    """
    3x3 rotation matrix → (roll, pitch, yaw) deg.
    Mirobot은 R = Rz(yaw) · Ry(pitch) · Rx(roll) 규칙(ZYX Euler angle)으로 report한다.
    """
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    pitch = np.degrees(np.arcsin(-R[2, 0]))
    roll = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    return np.array([roll, pitch, yaw])


# ---------- Step 1-4: 전체 data로 검증 ----------

if __name__ == "__main__":
    with open("data/fk_samples.csv") as f:
        rows = list(csv.DictReader(f))

    pos_errors, ori_errors = [], []
    for r in rows:
        joints = [float(r[k]) for k in ("j1", "j2", "j3", "j4", "j5", "j6")]
        robot_pos = np.array([float(r[k]) for k in ("x", "y", "z")])
        robot_rpy = np.array([float(r[k]) for k in ("roll", "pitch", "yaw")])

        T = fk_6axis(*joints)
        my_pos = T[:3, 3]
        my_rpy = matrix_to_rpy(T[:3, :3])

        pos_err = np.linalg.norm(my_pos - robot_pos)
        ori_err = np.max(np.abs(my_rpy - robot_rpy))
        pos_errors.append(pos_err)
        ori_errors.append(ori_err)

        print(f"joints={np.round(joints, 2)}")
        print(f"  position  mine={np.round(my_pos, 3)}  robot={robot_pos}  error={pos_err:.3f} mm")
        print(f"  rpy       mine={np.round(my_rpy, 3)}  robot={robot_rpy}  error={ori_err:.3f} deg")

    print(f"\nMax position error:    {max(pos_errors):.3f} mm")
    print(f"Max orientation error: {max(ori_errors):.3f} deg")
