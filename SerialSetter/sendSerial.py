import serial
import time

# ------------------------
# 串口配置
# ------------------------
PORT = "/dev/tty.usbmodem21403"  # 替换为你的串口号
BAUDRATE = 19200                  # 文档要求固定波特率
TIMEOUT = 1                        # 读取超时时间（秒）

# ------------------------
# 指令列表
# ------------------------
commands = [
    "m10.0",   # 模式1，移动到10度
    "m20.0",   # 模式1，移动到20度
    "r",       # 停止电机
]

# ------------------------
# 打开串口
# ------------------------
ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

try:
    for cmd in commands:
        # 文档要求加 \n\r
        full_cmd = cmd + "\n\r"
        ser.write(full_cmd.encode('ascii'))
        print(f"Sent: {full_cmd.strip()}")
        
        # 等待电机执行，避免命令冲突
        time.sleep(0.5)

finally:
    ser.close()
    print("Serial port closed.")