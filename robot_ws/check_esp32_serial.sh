#!/bin/bash
# ESP32 串口连接诊断脚本

echo "========================================="
echo "ESP32 串口连接诊断"
echo "========================================="
echo ""

echo "1. 检查容器内串口设备:"
echo "---"
ls -l /dev/tty* 2>/dev/null | grep -E "ttyUSB|ttyACM|ttyS6" || echo "  未找到 ttyUSB*/ttyACM* 设备"
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
EOF
echo ""

echo "========================================="
echo "诊断建议:"
echo "========================================="
echo ""
echo "如果未发现 ttyUSB* 设备,请按以下步骤操作:"
echo ""
echo "【Windows 宿主机 PowerShell (管理员)】"
echo "  1. 查看 USB 设备:"
echo "     usbipd list"
echo ""
echo "  2. 找到 ESP32 设备 (通常显示为 Silicon Labs CP210x 或 CH340)"
echo "     记下 BUSID (例如: 2-3)"
echo ""
echo "  3. 绑定设备 (只需一次):"
echo "     usbipd bind --busid <BUSID>"
echo ""
echo "  4. 连接到 WSL (每次重启后需要):"
echo "     usbipd attach --wsl --busid <BUSID>"
echo ""
echo "  5. 重新进入容器,应该能看到 /dev/ttyUSB0"
echo ""
echo "【树莓派环境】"
echo "  树莓派上直接连接 USB,无需额外配置"
echo "  设备应自动显示为 /dev/ttyUSB0"
echo ""
