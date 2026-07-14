import serial
import time

# 修改为你的实际串口号，Windows下通常是 COM3、COM4 等
PORT = 'COM3'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(1.5)  # 等待ESP32启动

def send_move(steps, direction=1, delay_ms=10):
    """
    steps: 步数
    direction: 1=正转, 0=反转
    delay_ms: 每步间隔(ms)，越大越慢，建议 5~50
    """
    cmd = f'MOVE,{steps},{direction},{delay_ms}\n'
    ser.write(cmd.encode())
    resp = ser.readline().decode().strip()
    print(f'发送: {cmd.strip()} -> 响应: {resp}')
    return resp

if __name__ == '__main__':
    print('连接成功，开始控制...')

    # 示例：正转200步（1圈，细分默认16时需3200步）
    send_move(steps=200, direction=1, delay_ms=15)
    time.sleep(0.5)

    # 示例：反转200步
    send_move(steps=200, direction=0, delay_ms=15)

    ser.close()
