import time
import sys
import math
import signal

# --- 路径配置 ---
sys.path.append('../lib') 
from unitree_actuator_sdk import *

# 请确认你的串口别名
SERIAL_PORT = '/dev/unitree_motor' 
REDUCTION_RATIO = 6.33

# --- 仿照官方示例的“超静音”参数 ---
# 官方示例: kp=0.0, kd=0.01 (纯速度模式)
# 我们的微调: kp=0.5, kd=0.05 (极软的位置模式)
KP_SILENT = 0.5   # 极小！只有之前的 1/10，电机几乎没有力气，非常安静
KD_SILENT = 0.05  # 极小！接近官方示例量级
TEST_AMPLITUDE_ROTOR = 2.0  # 转子端摆动幅度
TEST_FREQUENCY = 0.5        # 0.5Hz 慢速摆动

serial = SerialPort(SERIAL_PORT)
cmd = MotorCmd()
data = MotorData()

# 初始化类型
cmd.motorType = MotorType.GO_M8010_6
data.motorType = MotorType.GO_M8010_6

print("正在连接电机...")

# 1. 安全启动，先读取位置
for i in range(50):
    cmd.mode = 1 # FOC模式
    cmd.kp = 0.0; cmd.kd = 0.0; cmd.tau = 0.0
    serial.sendRecv(cmd, data)

if data.merror != 0:
    print(f"[错误] 电机异常码: {data.merror}")
    sys.exit()

start_rotor_pos = data.q
print(f"初始位置: {start_rotor_pos:.4f}")
print("开始超静音测试 (仿官方参数)...")

start_time = time.time()
target_loop_time = 0.001 # 1000Hz

try:
    while True:
        loop_start = time.time()
        t = loop_start - start_time
        
        # 1. 计算正弦位置 q
        sine_val = math.sin(2 * math.pi * TEST_FREQUENCY * t)
        target_q = start_rotor_pos + TEST_AMPLITUDE_ROTOR * sine_val
        
        # 2. 计算余弦速度 dq (关键！因为Kp很小，必须靠dq带着走)
        cosine_val = math.cos(2 * math.pi * TEST_FREQUENCY * t)
        target_dq = TEST_AMPLITUDE_ROTOR * (2 * math.pi * TEST_FREQUENCY) * cosine_val
        
        # 3. 发送指令 (参数极低)
        cmd.mode = 1
        cmd.q = target_q
        cmd.dq = target_dq  # 给定速度前馈，顺势而为
        cmd.kp = KP_SILENT  # 0.5 (很软)
        cmd.kd = KD_SILENT  # 0.05 (很小)
        cmd.tau = 0.0

        serial.sendRecv(cmd, data)

        # 4. 1000Hz 稳频
        while (time.time() - loop_start) < target_loop_time:
            pass

except KeyboardInterrupt:
    print("\n停止中...")
    cmd.mode = 1
    cmd.kp = 0.0
    cmd.kd = 0.0
    cmd.tau = 0.0
    for _ in range(20):
        serial.sendRecv(cmd, data)
    print("已停止。")