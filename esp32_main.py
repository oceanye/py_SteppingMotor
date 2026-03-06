import machine
import time
import sys

PUL_PIN = 18
DIR_PIN = 19

pul = machine.Pin(PUL_PIN, machine.Pin.OUT)
dir_pin = machine.Pin(DIR_PIN, machine.Pin.OUT)

def move(steps, direction, delay_ms=10):
    """
    steps: 脉冲数（步数）
    direction: 1=正转, 0=反转
    delay_ms: 每步间隔毫秒（越大越慢）
    """
    dir_pin.value(direction)
    time.sleep_ms(1)  # 方向稳定延时
    for _ in range(steps):
        pul.value(1)
        time.sleep_us(50)
        pul.value(0)
        time.sleep_ms(delay_ms)

def parse_and_run(cmd):
    """
    指令格式: MOVE,steps,direction,delay_ms
    示例: MOVE,200,1,10
    """
    parts = cmd.strip().split(',')
    if parts[0] == 'MOVE' and len(parts) == 4:
        steps = int(parts[1])
        direction = int(parts[2])
        delay_ms = int(parts[3])
        move(steps, direction, delay_ms)
        print('OK')
    else:
        print('ERR:unknown command')

print('ESP32 StepperMotor Ready')

while True:
    if sys.stdin in []:
        pass
    line = sys.stdin.readline()
    if line:
        parse_and_run(line)
