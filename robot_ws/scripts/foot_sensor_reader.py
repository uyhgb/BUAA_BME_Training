import serial
import time

# --- 配置 ---
SERIAL_PORT = '/dev/foot_usb'  # 你的串口号
BAUD_RATE = 115200            # 波特率
FRAME_HEADER = 0xAA           # 帧头
FRAME_LEN = 39                # 帧长: 1(头)+1(ID)+36(数据)+1(校验) = 39字节

# 18个感应点的名称映射 (根据手册P4右脚图示)
# 注意：数组索引是从0开始的，所以 点1 对应 index 0
SENSOR_MAPPING = {
    0: "大拇指(Toe)",
    1: "二拇指",
    2: "脚跟外侧(Heel-Out)",
    3: "脚跟内侧(Heel-In)",
    # ... 其他点位可以根据需要添加
    17: "前掌外侧" 
}

# 关键点索引 (用于快速查看脚跟和前掌状态)
# 右脚关键点:
# 脚跟: 点3 (idx 2), 点4 (idx 3), 点9 (idx 8), 点10 (idx 9)
# 前掌: 点1 (idx 0), 点2 (idx 1) ...
HEEL_INDICES = [2, 3, 8, 9] 

def verify_checksum(frame):
    """
    校验和验证：前38字节之和的低八位 == 第39字节
    """
    if len(frame) < FRAME_LEN:
        return False
    # 计算校验和
    checksum_calc = sum(frame[:38]) & 0xFF
    checksum_recv = frame[38]
    return checksum_calc == checksum_recv

def parse_pressure_values(payload):
    """
    将36字节的载荷解析为18个压力值 (单位: g)
    """
    values = []
    for i in range(0, 36, 2):
        # 高位在前，低位在后
        val = (payload[i] << 8) | payload[i+1]
        values.append(val)
    return values

def main():
    try:
        # 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ 连接成功: {SERIAL_PORT}")
        print("等待数据流... (请按压传感器)")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return

    buffer = bytearray()
    last_print_time = 0

    while True:
        try:
            # 1. 读取数据到缓冲区
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)

            # 2. 处理缓冲区中的数据帧
            while len(buffer) >= FRAME_LEN:
                # 寻找帧头 0xAA
                if buffer[0] != FRAME_HEADER:
                    # 如果不是帧头，移除这个字节，继续找
                    buffer.pop(0)
                    continue
                
                # 提取潜在的一帧数据
                frame = buffer[:FRAME_LEN]
                
                # 校验这一帧
                if verify_checksum(frame):
                    # --- 解析有效数据 ---
                    device_id = frame[1] # 01=左脚, 02=右脚
                    payload = frame[2:38] # 中间36字节是压力数据
                    
                    pressures = parse_pressure_values(payload)
                    
                    # 计算脚跟总压力
                    heel_force = sum([pressures[i] for i in HEEL_INDICES])
                    total_force = sum(pressures)

                    # --- 打印输出 (每0.2秒打印一次，防止刷屏) ---
                    if time.time() - last_print_time > 0.2:
                        foot_side = "右脚" if device_id == 0x02 else "左脚"
                        
                        print("-" * 50)
                        print(f"[{foot_side}] 总压力: {total_force} g")
                        print(f"  > 脚跟压力: {heel_force} g")
                        
                        # 如果有压力，打印详细点位
                        if total_force > 0:
                            print(f"  > 原始数据(前5点): {pressures[:5]}")
                        else:
                            print("  > 状态: 悬空/无压力")
                            
                        last_print_time = time.time()

                    # 从缓冲区移除已处理的这一帧
                    buffer = buffer[FRAME_LEN:]
                
                else:
                    # 校验失败，说明可能是假帧头，移除第一个字节重试
                    # print("校验失败，丢弃帧")
                    buffer.pop(0)
            
            # 降低CPU占用
            time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n停止程序")
            ser.close()
            break
        except Exception as e:
            print(f"运行错误: {e}")
            break

if __name__ == "__main__":
    main()