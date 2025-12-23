import time
import sys
import os
import csv
import datetime
import signal

# --- 路径配置 ---
sys.path.append('../lib')
# 根据实际情况导入 SDK，如果你的环境里是 unitree_actuator_sdk 保持不变
from unitree_actuator_sdk import * # --- 全局配置 ---
SERIAL_PORT = '/dev/ttyUSB0'
DATA_DIR = '/home/appendix41/unitree_actuator_sdk-main/data'
REDUCTION_RATIO = 6.33  # GO-M8010-6 减速比 

# --- 准备 CSV 文件 ---
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

# 使用时间戳命名文件，防止覆盖
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = os.path.join(DATA_DIR, f"trajectory_{timestamp}.csv")

# 初始化串口和变量
serial = SerialPort(SERIAL_PORT)
cmd = MotorCmd()
data = MotorData()

# 【新增】必须在循环外显式初始化类型
cmd.motorType = MotorType.GO_M8010_6
data.motorType = MotorType.GO_M8010_6

# 控制打印频率的计数器
print_count = 0

print(f"开始采集！数据将保存至: {csv_filename}")
print("按 Ctrl+C 停止采集并保存文件...")

try:
    with open(csv_filename, 'w', newline='') as csvfile:
        # 定义表头：时间戳, 转子角度, 转子速度, 关节角度(输出轴), 关节速度(输出轴), 力矩
        fieldnames = ['timestamp', 'rotor_q', 'rotor_dq', 'joint_q', 'joint_dq', 'tau']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        start_time = time.time()

        while True:
            # --- 1. 发送零力矩指令 [cite: 490-495] ---
            cmd.motorType = MotorType.GO_M8010_6
            cmd.mode = 1      # FOC闭环模式
            cmd.id   = 0      # 你的电机ID
            cmd.q    = 0.0
            cmd.dq   = 0.0
            cmd.kp   = 0.0    # 刚度为0
            cmd.kd   = 0.0    # 阻尼为0 (如果电机抖动，可设为 0.01-0.05)
            cmd.tau  = 0.0    # 前馈力矩为0

            # 发送并接收
            serial.sendRecv(cmd, data)

            # --- 2. 数据处理与保存 ---
            current_time = time.time() - start_time
            
            # 换算到输出轴（关节）端 
            # 注意：这里假设 data.q 是转子端弧度。
            joint_q = data.q / REDUCTION_RATIO
            joint_dq = data.dq / REDUCTION_RATIO

            # 写入 CSV
            writer.writerow({
                'timestamp': f"{current_time:.4f}",
                'rotor_q': f"{data.q:.4f}",
                'rotor_dq': f"{data.dq:.4f}",
                'joint_q': f"{joint_q:.4f}",   # 这是你做步态分析需要的真实角度
                'joint_dq': f"{joint_dq:.4f}",
                'tau': f"{data.tau:.4f}"       # 记录力矩以监测是否真的为0
            })

            # --- 3. 降低打印频率 ---
            # 原来的每次打印会严重拖慢循环，改为每 500 次循环打印一次状态
            print_count += 1
            if print_count >= 500:
                print(f"Time: {current_time:.2f}s | Joint Angle: {joint_q:.3f} rad | Temp: {data.temp}°C")
                print_count = 0

            # 保持高频循环，减少 sleep 时间或直接去掉
            # 200us sleep 加上串口IO和文件IO，实际周期可能在 1-2ms 左右
            time.sleep(0.0002)

except KeyboardInterrupt:
    print("\n[INFO] 采集停止，文件已保存。")
    print(f"文件路径: {csv_filename}")

except Exception as e:
    print(f"\n[ERROR] 发生错误: {e}")