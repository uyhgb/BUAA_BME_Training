import time
import sys
import math
import signal

# --- 路径配置 ---
sys.path.append('../lib') 
from unitree_actuator_sdk import * # --- 全局配置 ---
SERIAL_PORT = '/dev/ttyUSB0' # 请确认端口是否为你修改后的 /dev/unitree_motor
REDUCTION_RATIO = 6.33       # 减速比

# --- 安全参数配置 (关键) ---
TEST_AMPLITUDE_ROTOR = 1.0   # 转子端摆动幅度(弧度)。1.0 rad / 6.33 ≈ 9度 (非常小，很安全)
TEST_FREQUENCY = 0.5         # 摆动频率 (Hz)，0.5代表2秒摆一个来回，很慢
KP_SOFT = 2.0                # 刚度系数：设得很小 (正常可能是 20-60)，遇到阻力会"软"掉
KD_SOFT = 0.5                # 阻尼系数：防止抖动

# 初始化
serial = SerialPort(SERIAL_PORT)
cmd = MotorCmd()
data = MotorData()

# 显式初始化类型
cmd.motorType = MotorType.GO_M8010_6
data.motorType = MotorType.GO_M8010_6

print("正在连接电机...")

# --- 1. 安全启动：先读取当前位置 ---
# 必须先读几帧数据，确保 data.q 不是默认的 0，而是真实位置
for i in range(50):
    cmd.mode = 1      # 闭环模式
    cmd.kp = 0.0      # 刚度0，先不发力
    cmd.kd = 0.0
    cmd.tau = 0.0
    serial.sendRecv(cmd, data)
    time.sleep(0.002)

if data.merror != 0:
    print(f"[错误] 电机异常，错误码: {data.merror}")
    sys.exit()

# 记录启动时的初始位置 (以这个位置为中心摆动)
start_rotor_pos = data.q
print(f"--- 初始化完成 ---")
print(f"初始转子位置: {start_rotor_pos:.4f} rad")
print(f"即将开始微幅摆动测试 (幅度: ±{TEST_AMPLITUDE_ROTOR/REDUCTION_RATIO*57.3:.1f}度)")
print("注意观察电机旋转方向！")
print("按 Ctrl+C 立即停止 (电机将卸力)")
time.sleep(1)

start_time = time.time()

try:
    while True:
        t = time.time() - start_time
        
        # --- 2. 计算微幅正弦轨迹 ---
        # 目标位置 = 初始位置 + (幅度 * sin(wt))
        # 这样电机永远只在初始位置附近晃悠，不会乱跑
        target_rotor_pos = start_rotor_pos + TEST_AMPLITUDE_ROTOR * math.sin(2 * math.pi * TEST_FREQUENCY * t)
        
        # --- 3. 发送控制指令 ---
        cmd.mode = 1
        cmd.q = target_rotor_pos   # 设定目标位置
        cmd.dq = 0.0
        cmd.kp = KP_SOFT           # 使用软刚度
        cmd.kd = KD_SOFT           # 使用软阻尼
        cmd.tau = 0.0

        serial.sendRecv(cmd, data)

        # --- 4. 打印调试信息 ---
        # 打印 "目标值" vs "实际值"
        # 如果 目标变大，实际也变大 -> 方向正确 (正向)
        # 如果 目标变大，实际变小 -> 方向相反
        print(f"目标(Target): {target_rotor_pos:.3f} | 实际(Real): {data.q:.3f} | 差值: {target_rotor_pos - data.q:.3f}")
        
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n[停止] 正在卸力...")
    # 退出时发送全0指令，让电机松弛
    cmd.mode = 1
    cmd.kp = 0.0
    cmd.kd = 0.0
    cmd.tau = 0.0
    for _ in range(10):
        serial.sendRecv(cmd, data)
        time.sleep(0.002)
    print("测试结束，电机已释放。")

except Exception as e:
    print(f"\n[错误] {e}")