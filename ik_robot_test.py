"""
Step 2-3: 내 IK vs Mirobot firmware IK 비교 (실제 robot 사용)

1. set_tool_pose()로 robot이 "자기 IK"로 target에 가게 한 뒤 → joint angle 읽기
2. 같은 target을 내 ik_mirobot()으로 계산
3. 두 joint angle을 비교

안전: table과 부딪히지 않도록 z는 100mm 이상만 사용.
"""
import time

import numpy as np
from wlkata_mirobot import WlkataMirobot

from find_port import find_mirobot_port
from ik_mirobot import ik_mirobot

TARGETS = [  # (x, y, z, yaw)
    (200, 0, 150, 0),
    (180, 80, 120, 0),
    (150, -100, 120, 30),
]


def wait_until_pose(arm, target_xyz, tol=0.5, timeout=15.0):
    """Tool 끝이 target position에 실제로 도착할 때까지 polling (race condition 방지)."""
    start = time.time()
    while time.time() - start < timeout:
        arm.get_status()
        p = arm.pose
        if np.linalg.norm(np.array([p.x, p.y, p.z]) - target_xyz) <= tol and arm.state == "Idle":
            return
        time.sleep(0.1)
    raise TimeoutError(f"Target {target_xyz}에 도착하지 못함")


def main():
    arm = WlkataMirobot(portname=find_mirobot_port())
    arm.home()

    for x, y, z, yaw in TARGETS:
        arm.set_tool_pose(x, y, z, roll=0.0, pitch=0.0, yaw=yaw)
        wait_until_pose(arm, np.array([x, y, z]))

        a = arm.angle
        robot_j = np.array([a.a1, a.a2, a.a3, a.a4, a.a5, a.a6])
        my_j = ik_mirobot(x, y, z, yaw)
        diff = np.abs(robot_j - my_j)

        print(f"target=({x}, {y}, {z}, yaw={yaw})")
        print(f"  robot IK = {np.round(robot_j, 2)}")
        print(f"  my IK    = {np.round(my_j, 2)}")
        print(f"  max diff = {diff.max():.3f} deg")

    arm.home()


if __name__ == "__main__":
    main()
