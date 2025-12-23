import sys
import time
sys.path.append('../lib') # 确保能找到 SDK
from unitree_actuator_sdk import *

SERIAL_PORT = '/dev/ttyUSB0'

def scan_motor_id():
    serial = SerialPort(SERIAL_PORT)
    cmd = MotorCmd()
    data = MotorData()
    
    # 初始化类型
    cmd.motorType = MotorType.GO_M8010_6
    data.motorType = MotorType.GO_M8010_6

    print("开始扫描电机 ID (0-15)...")
    
    for test_id in range(16):
        print(f"正在尝试 ping ID: {test_id} ...", end='', flush=True)
        
        cmd.id = test_id
        cmd.mode = 0  # 仅查询，不控制
        
        # 尝试发送接收几次，防止偶尔丢包
        for _ in range(5):
            serial.sendRecv(cmd, data)
            if data.merror != 0 or data.temp > 0 or data.q != 0:
                # 只要有数据回来（温度不是0，或者没有CRC报错），说明找到了
                print(f" [成功] 找到电机！ID = {test_id}")
                print(f"    状态: Temp={data.temp}°C, Pos={data.q}")
                return test_id
            time.sleep(0.005)
        
        print(" 无响应")
    
    print("扫描结束，未找到电机。请检查是否需要 sudo 权限或 latency_timer 设置。")
    return -1

if __name__ == '__main__':
    scan_motor_id()