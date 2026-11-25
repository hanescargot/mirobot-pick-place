from wlkata_mirobot import WlkataMirobot

from find_port import find_mirobot_port
# arm = WlkataMirobot(portname='/dev/cu.usbserial-14410')   # 자동 탐색이 블루투스 포트를 잡아서 직접 지정
arm = WlkataMirobot(portname=find_mirobot_port())

arm.home()              # 원점 복귀 (매번 처음에 필수)
arm.set_tool_pose(200, 20, 230)
print(arm.pose)
print(arm.status)