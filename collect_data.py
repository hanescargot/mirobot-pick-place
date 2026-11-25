"""
FK verification data collection.

Joint angle을 직접 지정해서 robot을 움직이고,
robot이 report하는 joint angle + end-effector pose를 CSV로 저장한다.
이 data가 나중에 내가 만든 FK code의 "정답(ground truth)"이 된다.
"""
import csv
import time

from wlkata_mirobot import WlkataMirobot
from find_port import find_mirobot_port

# 각 tuple = (J1, J2, J3, J4, J5, J6), 단위 degree
# Homing 직후 자세(0,0,0,0,0,0)에서 시작해 joint를 하나씩, 마지막엔 여러 개를 같이 움직인다.
JOINT_TARGETS = [
    (0, 0, 0, 0, 0, 0),
    (30, 0, 0, 0, 0, 0),     # J1만: base 회전
    (0, 20, 0, 0, 0, 0),     # J2만: shoulder
    (0, 0, -20, 0, 0, 0),    # J3만: elbow
    (0, 0, 0, 0, 30, 0),     # J5만: wrist pitch
    (20, 10, -10, 15, -20, 30),  # 여러 joint 동시
]

OUTPUT_CSV = "data/fk_samples.csv"
FIELDS = ["j1", "j2", "j3", "j4", "j5", "j6", "x", "y", "z", "roll", "pitch", "yaw"]


def wait_until_reached(arm, target, tol=0.1, timeout=15.0):
    """
    Robot이 target joint angle에 실제로 도착할 때까지 status를 polling한다.

    SDK의 wait_idle은 command 직후 robot이 아직 움직이기 전의 "Idle"을 보고
    바로 return해버리는 경우가 있다 (race condition).
    그래서 "state == Idle" 뿐 아니라 "angle이 target과 tol 이내"까지 확인한다.
    """
    start = time.time()
    while time.time() - start < timeout:
        arm.get_status()
        ang = arm.angle
        current = (ang.a1, ang.a2, ang.a3, ang.a4, ang.a5, ang.a6)
        reached = all(abs(c - t) <= tol for c, t in zip(current, target))
        if reached and arm.state == "Idle":
            return
        time.sleep(0.1)
    raise TimeoutError(f"Target {target}에 {timeout}s 안에 도착하지 못함. 현재: {current}")


def main():
    arm = WlkataMirobot(portname=find_mirobot_port())
    arm.home()

    rows = []
    for target in JOINT_TARGETS:
        arm.set_joint_angle({i + 1: angle for i, angle in enumerate(target)})
        wait_until_reached(arm, target)  # 실제로 도착할 때까지 대기

        ang, pose = arm.angle, arm.pose
        row = {
            "j1": ang.a1, "j2": ang.a2, "j3": ang.a3,
            "j4": ang.a4, "j5": ang.a5, "j6": ang.a6,
            "x": pose.x, "y": pose.y, "z": pose.z,
            "roll": pose.a, "pitch": pose.b, "yaw": pose.c,
        }
        rows.append(row)
        print(row)

    arm.home()  # 끝나면 안전한 자세로 복귀

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved {len(rows)} samples to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
