# USB 串口连接到 WSL2 完整流程

## 方案: 使用 usbipd-win 将 Windows USB 串口设备连接到 WSL2

---

## 📋 完整步骤

### 第一步: 在 Windows 安装 usbipd-win

**在 Windows PowerShell (管理员权限) 运行:**

```powershell
# 方法1: 使用 winget (Windows 10/11 自带)
winget install --interactive --exact dorssel.usbipd-win

# 方法2: 或者从 GitHub 下载安装
# 访问: https://github.com/dorssel/usbipd-win/releases
# 下载并安装最新版 .msi 文件
```

**安装完成后重启 PowerShell**

---

### 第二步: 在 WSL2 安装客户端工具

**在 WSL2 终端 (不是 Docker 容器内) 运行:**

```bash
# 更新包列表
sudo apt update

# 安装 USB/IP 工具和硬件数据库
sudo apt install linux-tools-generic hwdata -y

# 配置 usbip 命令
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

---

### 第三步: 在 Windows 查看并绑定 USB 设备

**在 Windows PowerShell (管理员) 运行:**

#### 1. 列出所有 USB 设备

```powershell
usbipd list
```

**输出示例:**
```
BUSID  VID:PID    DEVICE                                            STATE
1-4    10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM7)    Not shared
2-1    046d:c52b  USB 输入设备                                      Not shared
3-2    0781:5567  USB 大容量存储设备                                Not shared
```

#### 2. 找到你的 IMU 设备

- 当前项目的 IMU 在 **COM7**
- 对应的 BUSID 为 **1-4** (示例,根据实际输出确定)

#### 3. 绑定设备 (只需做一次)

```powershell
# 将 1-4 替换为你的实际 BUSID
usbipd bind --busid 1-4
```

#### 4. 连接设备到 WSL2

```powershell
# 将 1-4 替换为你的实际 BUSID
usbipd attach --wsl --busid 1-4
```

**成功提示:** `Device attached successfully`

---

### 第四步: 在 WSL2 验证设备

**在 WSL2 终端运行:**

```bash
# 查看是否出现 USB 串口设备
ls -l /dev/ttyUSB* /dev/ttyACM*

# 成功时可能显示:
# crw-rw---- 1 root dialout 188, 0 Dec  5 16:30 /dev/ttyUSB0
# 或
# crw-rw---- 1 root dialout 166, 0 Dec  5 16:30 /dev/ttyACM0
```

**确定设备名称:**
- CP210x, CH340 芯片通常显示为 `/dev/ttyUSB0`
- Arduino, STM32 等通常显示为 `/dev/ttyACM0`

---

### 第五步: 在 Docker 容器内验证

**在 Docker 容器终端 (VS Code 终端) 运行:**

```bash
# 查看串口设备
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# 测试读取数据 (3秒后自动停止)
timeout 3 cat /dev/ttyUSB0  # 或 /dev/ttyACM0

# 如果看到乱码或二进制数据,说明连接成功
```

---

### 第六步: 更新 ROS2 配置

**修改 IMU 配置文件:**

```bash
# 编辑配置文件
vim /workspace/robot_ws/src/imu_serial_publisher/config/imu_params.yaml
```

**更新串口设备名:**

```yaml
# 串口参数
serial_port: "/dev/ttyUSB0"  # 根据第四步确定的设备名修改
baud_rate: 115200

# 坐标系
frame_id: "imu_link"

# 发布频率 (Hz)
publish_rate: 20.0
```

---

### 第七步: 编译并运行 ROS2 节点

**在 Docker 容器内运行:**

```bash
# 1. 进入工作目录
cd /workspace

# 2. Source ROS2 环境
source /opt/ros/humble/setup.bash

# 3. 编译工作空间 (如果代码有修改)
colcon build

# 4. Source 本地工作空间
source install/setup.bash

# 5. 运行 IMU 发布节点
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args \
  --params-file robot_ws/src/imu_serial_publisher/config/imu_params.yaml

# 或者使用 launch 文件
ros2 launch imu_serial_publisher imu_publisher.launch.py
```

---

### 第八步: 验证数据发布

**在新的终端窗口运行:**

```bash
# Source 环境
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 查看话题列表
ros2 topic list

# 应该看到:
# /imu/data
# /imu/mag

# 查看 IMU 数据
ros2 topic echo /imu/data

# 查看发布频率
ros2 topic hz /imu/data
```

---

## 🔄 日常使用流程

### 每次重启 Windows 或重新连接设备后:

**在 Windows PowerShell (管理员) 运行:**

```powershell
# 连接设备
usbipd attach --wsl --busid 1-4

# 查看连接状态
usbipd list
# 状态应该显示: Attached
```

### 断开连接 (如需在 Windows 使用该设备):

```powershell
# 断开设备
usbipd detach --busid 1-4

# 或者从 WSL2 断开所有设备
usbipd detach --all
```

---

## ⚠️ 重要注意事项

### 1. 权限要求
- Windows PowerShell 必须以**管理员权限**运行
- WSL2 中的 sudo 命令需要密码

### 2. 设备访问互斥
- USB 设备连接到 WSL2 后,**Windows 无法同时访问**
- 如需在 Windows 使用,必须先 `detach`

### 3. Docker 配置要求
- `docker-compose.yml` 需要 `privileged: true` (已配置 ✅)
- 设备会自动映射到容器内

### 4. 重启后重新连接
- **每次 Windows 重启**需要重新 `attach`
- **每次拔插 USB 设备**需要重新 `attach`

### 5. 设备名称可能变化
- 首次连接后,设备名通常固定
- 如果连接多个同类设备,编号可能递增 (ttyUSB0, ttyUSB1...)

---

## 🐛 常见问题排查

### 问题 1: `usbipd: command not found`

**解决:**
- 确认已安装 usbipd-win
- 重启 PowerShell
- 检查 PATH 环境变量

### 问题 2: `access denied` 或权限错误

**解决:**
- 确保 PowerShell 以管理员权限运行
- 在 WSL2 中将用户添加到 dialout 组:
  ```bash
  sudo usermod -a -G dialout $USER
  ```

### 问题 3: 设备已连接但 WSL2 看不到

**解决:**
```bash
# 在 WSL2 中检查内核模块
lsmod | grep usbserial

# 如果没有,手动加载
sudo modprobe usbserial
sudo modprobe ftdi_sio    # FTDI 芯片
sudo modprobe cp210x      # CP210x 芯片
sudo modprobe ch341       # CH340/CH341 芯片
```

### 问题 4: Docker 容器内看不到设备

**解决:**
```bash
# 检查 docker-compose.yml 配置
# 确保有 privileged: true

# 或者添加设备映射
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

### 问题 5: `Input/output error` 读取串口

**解决:**
```bash
# 检查设备是否被其他进程占用
lsof /dev/ttyUSB0

# 重新配置串口参数
stty -F /dev/ttyUSB0 115200 raw -echo

# 检查波特率是否正确
```

---

## 📚 参考资料

- [微软官方文档: 连接 USB 设备到 WSL](https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb)
- [usbipd-win GitHub 仓库](https://github.com/dorssel/usbipd-win)
- [ROS2 串口通信教程](https://docs.ros.org/en/humble/index.html)

---

## ✅ 快速检查清单

- [ ] Windows 已安装 usbipd-win
- [ ] WSL2 已安装 linux-tools-generic
- [ ] usbipd list 能看到设备
- [ ] usbipd bind 成功
- [ ] usbipd attach 成功
- [ ] WSL2 能看到 /dev/ttyUSB0 或 /dev/ttyACM0
- [ ] Docker 容器内能看到串口设备
- [ ] 串口配置文件已更新
- [ ] ROS2 节点能成功读取数据

---

**当前项目配置:**
- IMU 串口: COM7 (Windows) → /dev/ttyUSB0 或 /dev/ttyACM0 (WSL2/Docker)
- 波特率: 115200
- 配置文件: `/workspace/robot_ws/src/imu_serial_publisher/config/imu_params.yaml`

**更新日期:** 2025-12-05
