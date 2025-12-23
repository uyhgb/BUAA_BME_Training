import serial
import time


class GO_M8010_6:
    def __init__(self, port='COM3', baudrate=115200):
        """初始化电机控制器

        Args:
            port: 串口号，Windows通常为COMx，Linux为/dev/ttyUSBx
            baudrate: 波特率，需与电机设置一致
        """
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_1
        self.ser.timeout = 0.1  # 读取超时时间
        self.ser.write_timeout = 1  # 写入超时时间

        # 电机ID，可根据实际情况修改
        self.motor_id = 0x01

    def connect(self):
        """连接到电机"""
        try:
            if not self.ser.is_open:
                self.ser.open()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开与电机的连接"""
        if self.ser.is_open:
            self.ser.close()

    def send_command(self, cmd, params=[]):
        """发送命令到电机

        Args:
            cmd: 命令代码
            params: 参数列表

        Returns:
            电机返回的数据，如果失败则返回None
        """
        if not self.ser.is_open:
            print("未连接到电机")
            return None

        # 构建数据包 (具体格式需参考电机通信协议)
        # 这里的格式仅为示例，实际需根据电机手册修改
        packet = [self.motor_id, cmd, len(params)] + params

        # 计算校验和 (具体算法需参考电机通信协议)
        checksum = sum(packet) & 0xFF
        packet.append(checksum)

        # 发送数据
        try:
            self.ser.write(bytearray(packet))
            time.sleep(0.01)  # 等待响应

            # 读取响应
            response = self.ser.read(100)  # 读取最多100字节
            return response
        except Exception as e:
            print(f"发送命令失败: {e}")
            return None

    def set_speed(self, speed):
        """设置电机速度

        Args:
            speed: 速度值，范围需参考电机手册
        """
        # 0x02 为设置速度命令示例，实际命令需参考电机手册
        # 将速度转换为两个字节 (假设范围-32768到32767)
        speed_bytes = [(speed >> 8) & 0xFF, speed & 0xFF]
        return self.send_command(0x02, speed_bytes)

    def stop(self):
        """停止电机"""
        # 0x03 为停止命令示例，实际命令需参考电机手册
        return self.send_command(0x03)

    def get_position(self):
        """获取电机当前位置"""
        # 0x04 为获取位置命令示例，实际命令需参考电机手册
        response = self.send_command(0x04)
        if response and len(response) >= 4:  # 假设响应包含位置信息
            # 解析位置数据 (具体解析方式需参考电机手册)
            position = (response[2] << 8) | response[3]
            return position
        return None


if __name__ == "__main__":
    # 初始化电机控制器，注意修改串口号为实际使用的端口
    motor = GO_M8010_6(port='COM3')

    # 连接电机
    if motor.connect():
        print("电机连接成功")

        try:
            # 简单控制示例
            print("设置速度为500")
            motor.set_speed(500)
            time.sleep(2)  # 运行2秒

            print("获取当前位置")
            pos = motor.get_position()
            print(f"当前位置: {pos}")

            print("设置速度为-500 (反向)")
            motor.set_speed(-500)
            time.sleep(2)  # 反向运行2秒

            print("停止电机")
            motor.stop()

        finally:
            # 确保断开连接
            motor.disconnect()
            print("电机已断开连接")
    else:
        print("无法连接到电机，请检查端口和连接")
