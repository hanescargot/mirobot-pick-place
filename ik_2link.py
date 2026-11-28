"""
Step 2-1: 2-link planar IK (analytic)

Target (x, y)가 주어졌을 때, 2-link arm의 joint angle (θ1, θ2)를 구한다.
- θ1: x축에서 첫 번째 link까지의 angle (반시계 +)
- θ2: 첫 번째 link에서 두 번째 link까지 "더 꺾인" angle (반시계 +)

FK (정답 검증용, 이미 구현됨):
    x = L1·cos(θ1) + L2·cos(θ1 + θ2)
    y = L1·sin(θ1) + L2·sin(θ1 + θ2)
"""
import numpy as np

L1 = 100.0
L2 = 80.0


def fk_2link(t1, t2):
    """θ1, θ2 (rad) → (x, y). IK 결과를 검증할 때 쓴다."""
    x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
    y = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
    return np.array([x, y])


def ik_2link(x, y, elbow_sign=+1):
    """
    (x, y) → (θ1, θ2) in rad.
    elbow_sign = +1 또는 -1 로 두 solution 중 하나를 고른다.
    닿을 수 없는 target이면 ValueError를 raise 한다.

    Step 1: law of cosines로 cos(θ2) 구하기
        d² = x² + y²
        d² = L1² + L2² + 2·L1·L2·cos(θ2)
        → cos(θ2) = (d² − L1² − L2²) / (2·L1·L2)

    Step 2: cos(θ2)가 -1 ~ 1 범위 밖이면 → 닿을 수 없음 → ValueError

    Step 3: θ2 = elbow_sign · arccos(cos(θ2))

    Step 4: θ1 = (target 방향 angle) − (L1 방향과 target 방향 사이의 angle)
        θ1 = atan2(y, x) − atan2(L2·sin(θ2), L1 + L2·cos(θ2))
    """
    # 1. Law of cosines로 cos(θ2) 구하기
    d2 = x**2 + y**2
    cos_t2 = (d2 - L1**2 - L2**2) / (2 * L1 * L2)

    # 2. 범위 밖이면 그런 삼각형이 없음 → 닿을 수 없음
    if cos_t2 < -1 or cos_t2 > 1:
        raise ValueError(f"target ({x}, {y})은 reach 밖 (d = {np.sqrt(d2):.1f})")

    # 3. θ2: ± 부호로 두 solution 중 하나 선택
    t2 = elbow_sign * np.arccos(cos_t2)

    # 4. θ1 = target 방향 − 팔꿈치 때문에 어긋난 angle
    t1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))

    return t1, t2


if __name__ == "__main__":
    # 1) FK(IK(target)) == target 인지 확인 (self-check)
    targets = [(120, 80), (150, 0), (0, 150), (-90, 60), (60, -100)]
    for x, y in targets:
        for sign in (+1, -1):
            t1, t2 = ik_2link(x, y, sign)
            back = fk_2link(t1, t2)
            err = np.linalg.norm(back - np.array([x, y]))
            print(f"target=({x:4}, {y:4})  sign={sign:+d}  "
                  f"θ1={np.degrees(t1):7.2f}°  θ2={np.degrees(t2):7.2f}°  "
                  f"FK(IK)={np.round(back, 3)}  error={err:.6f}")

    # 2) 닿을 수 없는 target은 error가 나야 정상
    for x, y in [(200, 0), (5, 5)]:
        try:
            ik_2link(x, y)
            print(f"target=({x}, {y}): ❌ error가 나야 하는데 solution을 return함")
        except ValueError as e:
            print(f"target=({x}, {y}): ✅ unreachable → {e}")
