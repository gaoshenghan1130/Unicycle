import serial
import struct

# 打开 CANable 串口
ser = serial.Serial("/dev/tty.usbmodem205A3865464D1", 115200, timeout=1)

def parse_can_frame(frame_bytes):
    if len(frame_bytes) < 16:
        return None
    can_id = struct.unpack(">I", frame_bytes[0:4])[0]  # 大端解析 CAN ID
    dlc = frame_bytes[4]                                # 数据长度
    data = frame_bytes[5:5+dlc]                         # 数据字节
    return can_id, dlc, data

print("开始读取 CAN 帧...")
while True:
    frame = ser.read(16)   # 每次读取一帧 16 字节
    parsed = parse_can_frame(frame)
    if parsed:
        can_id, dlc, data = parsed
        print(f"CAN ID: {can_id:X}, DLC: {dlc}, DATA: {data.hex()}")