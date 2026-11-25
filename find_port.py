from serial.tools import list_ports

CH340_VID = 0x1A86 # Mirobot 안의 CH340 칩 VID

def find_mirobot_port():
    for p in list_ports.comports():
        if p.vid == CH340_VID:
            return p.device
    raise RuntimeError("Mirobot을 찾지 못했어요. USB 연결을 확인하세요.")