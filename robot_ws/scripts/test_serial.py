import serial
import time

# --- 配置 ---
SERIAL_PORT = '/dev/foot_usb'  # 请确认您的端口号
BAUD_RATE = 115200            # 足底传感器默认是 115200

def test_serial_send():
    try:
        # 1. 打开串口
        # timeout=1 表示读取时最多等1秒，write_timeout=1 表示发送堵塞最多等1秒
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=1)
        
        if ser.is_open:
            print(f"✅ 串口 {SERIAL_PORT} 打开成功！(波特率: {BAUD_RATE})")
        
        # --- 场景 A: 发送普通文本 (ASCII) ---
        # 很多设备收到非法指令没反应，但我们可以先发个简单的试试
        text_data = "Hello Raspberry Pi\r\n"
        print(f"--> 正在发送文本: {text_data.strip()}")
        ser.write(text_data.encode('utf-8')) # 必须编码成 bytes
        ser.flush() # 强制把缓冲区数据推出去，确保物理发送
        time.sleep(0.5) # 等待设备可能的回复
        
        # --- 场景 B: 发送足底传感器专用指令 (根据手册 P8) ---
        # 比如查询当前配置指令: SET=?
        # 注意：手册里有些指令不需要换行，有些可能需要。通常加个回车是安全的。
        cmd_data = "SET=?\r\n" 
        print(f"--> 正在发送传感器指令: {cmd_data.strip()}")
        ser.write(cmd_data.encode('utf-8'))
        ser.flush()
        
        # --- 3. 看看有没有回音 (接收测试) ---
        # 如果设备收到指令并回复，这里能读到
        print("... 正在监听回复 (2秒) ...")
        start_time = time.time()
        while time.time() - start_time < 2:
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                # 尝试解码打印，如果解不了就打 HEX
                try:
                    print(f"✅ 收到回复 (ASCII): {response.decode('utf-8').strip()}")
                except:
                    print(f"✅ 收到回复 (Hex): {response.hex().upper()}")
            time.sleep(0.1)

        ser.close()
        print("--- 测试结束，串口已关闭 ---")

    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        print("提示：\n1. 请检查 /dev/foot_usb 是否存在 (ls /dev/foot_usb)")
        print("2. 权限是否足够 (sudo chmod 666 /dev/foot_usb)")
        print("3. 设备是否被其他程序(如ROS)占用了？")

    except Exception as e:
        print(f"❌ 其他错误: {e}")

if __name__ == "__main__":
    test_serial_send()