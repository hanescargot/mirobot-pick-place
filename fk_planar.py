"""
Step 1-2: Side view (x-z plane) planar FK

Mirobot을 옆에서 보고, J2(shoulder)와 J3(elbow) 두 joint만 있는
2-link arm으로 단순화해서 tool 끝의 (x, z)를 계산한다.
J1(base 회전)은 마지막에 cos/sin으로 x, y에 나눠 준다.

Sign convention (data에서 확인):
  x는 오른쪽(앞), z는 위쪽으로 그렸을 때,
  J2, J3, J5의 + 방향은 "시계 방향" = 팔이 앞으로 숙여지는 방향.
"""
import numpy as np

# Mirobot geometry (mm), home 자세(모든 joint 0도) 기준
J2_POS = np.array([29.69, 127.0])   # base origin에서 본 J2(shoulder) 위치 (x, z)
UPPER_ARM = np.array([0.0, 108.0])  # J2 → J3: 위로 108
FOREARM = np.array([168.98, 20.0])  # J3 → J5(wrist): 앞으로 168.98, 위로 20
TOOL = np.array([0.0, -24.27])      # J5 → tool 끝: 아래로 24.27


def rot_cw(theta_deg):
    """
    2D rotation matrix (시계 방향 = Mirobot의 + 방향).
    TODO: 시계 방향 회전 matrix를 return 하세요.
    Hint: 반시계 방향은 [[cos, -sin], [sin, cos]]. 시계 방향은 θ 대신 -θ를 넣은 것.
    """
    t = np.radians(theta_deg)

    c, s = np.cos(t), np.sin(t)
    return np.array([[c, s],
                     [-s, c]])
    raise NotImplementedError("rot_cw를 구현하세요")


def fk_planar(j2, j3):
    """
    J2, J3 angle(deg) → tool 끝의 (x, z) in side view.

    TODO: 아래 순서로 구현하세요.
      1. upper arm 끝(J3 위치) = J2_POS + rot_cw(j2) @ UPPER_ARM
      2. forearm은 J2와 J3 회전을 "둘 다" 받는다 → 각도는 j2 + j3
      3. tool도 앞의 회전을 모두 받는다 (J5 = 0 가정)
    """
    p3 = J2_POS + rot_cw(j2) @ UPPER_ARM #J3(elbow) 위치
    p5 = p3 + rot_cw(j2 + j3) @ FOREARM #J5(wrist) 위치

    tip = p5 + rot_cw(j2 + j3) @ TOOL # tip 위치 

    return tip

    raise NotImplementedError("fk_planar를 구현하세요")


def fk_with_base(j1, j2, j3):
    """Side view 결과 (r, z)를 J1 회전으로 3D (x, y, z)로 확장."""
    r, z = fk_planar(j2, j3)
    t = np.radians(j1)
    return np.array([r * np.cos(t), r * np.sin(t), z])


if __name__ == "__main__":
    # (j1, j2, j3) → robot이 report한 (x, y, z)   [data/fk_samples.csv]
    tests = [
        ((0, 0, 0),       (198.67, 0.0, 230.72)),
        ((30.003, 0, 0),  (172.048, 99.345, 230.72)),
        ((0, 20, 0),      (223.954, 0.0, 166.67)),
        ((0, 0, -20.004), (189.939, 0.0, 288.785)),
    ]
    for joints, expected in tests:
        got = fk_with_base(*joints)
        err = np.linalg.norm(got - np.array(expected))
        print(f"joints={joints}  mine={np.round(got, 3)}  robot={expected}  error={err:.3f} mm")
