#!/bin/bash
# ESP32 串口连接诊断脚本

echo "========================================="
echo "ESP32 串口连接诊断"
echo "========================================="
echo ""

echo "1. 检查容器内串口设备:"
echo "---"
ls -l /dev/tty* 2>/dev/null | grep -E "ttyUSB|ttyACM" || echo "  未找到 ttyUSB*/ttyACM* 设备"
echo ""

echo "2. 检查 USB 设备 (如果在 WSL2 中):"
echo "---"
if command -v lsusb &> /dev/null; then
    lsusb | grep -E "CP210|CH340|Silicon|UART|Serial" || echo "  未找到常见 USB 转串口芯片"
else
    echo "  lsusb 命令不可用"
fi
echo ""

echo "3. Python serial 库测试:"
echo "---"
python3 << 'EOF'
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
if ports:
    print("  发现的串口设备:")
    for port in ports:
        print(f"    - {port.device}: {port.description}")
        if port.manufacturer:
            print(f"      制造商: {port.manufacturer}")
else:
    print("  未发现任何串口设备")
