# 树莓派 Docker 部署配置说明

## 环境差异

### 开发环境 (Windows + WSL2)
- **串口穿透方式**: usbipd-win
- **设备路径**: `/dev/ttyUSB0` (通过 usbipd 映射)
- **Docker 配置**: `privileged: true`
- **配置文件**: `imu_params_dev_wsl.yaml`

### 生产环境 (树莓派)
- **串口穿透方式**: 直接硬件连接
- **设备路径**: `/dev/ttyUSB0` (原生 USB 设备)
- **Docker 配置**: 需要显式映射设备
- **配置文件**: `imu_params_raspberry_pi.yaml`

---

## 树莓派 Docker Compose 配置

### 方案一: 使用 privileged 模式 (简单但权限较大)

```yaml
version: '3'
services:
  robot_controller:
    image: ros2-humble-dev:latest
    container_name: robot_controller
    privileged: true  # 允许访问所有设备
    volumes:
      - ./workspace:/workspace
    network_mode: "host"
    command: /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch robot_controller robot_system.launch.py"
```

### 方案二: 显式映射设备 (推荐,更安全)

```yaml
version: '3'
services:
  robot_controller:
    image: ros2-humble-dev:latest
    container_name: robot_controller
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # IMU 串口设备
      # 如果有多个串口设备:
      # - /dev/ttyUSB1:/dev/ttyUSB1
      # - /dev/ttyACM0:/dev/ttyACM0
    volumes:
      - ./workspace:/workspace
    network_mode: "host"
    command: /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch robot_controller robot_system.launch.py"
```

### 方案三: 使用固定设备 ID (最稳定)

```yaml
version: '3'
services:
  robot_controller:
    image: ros2-humble-dev:latest
    container_name: robot_controller
    devices:
      # 使用固定的设备 ID,避免设备号变化
      - /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0:/dev/ttyIMU
    volumes:
      - ./workspace:/workspace
      - /dev:/dev:ro  # 只读映射整个 /dev (用于查找设备)
    network_mode: "host"
    environment:
      - ROS_DOMAIN_ID=0
    command: /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 launch robot_controller robot_system.launch.py"
```

---

## 树莓派系统配置

### 1. 查看串口设备

```bash
# 列出所有 USB 串口
ls -l /dev/ttyUSB* /dev/ttyACM*

# 查看设备固定 ID (推荐使用)
ls -l /dev/serial/by-id/

# 示例输出:
# lrwxrwxrwx 1 root root 13 Dec  5 10:30 usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

### 2. 配置用户权限

```bash
# 将当前用户添加到 dialout 组 (串口访问权限)
sudo usermod -a -G dialout $USER

# 重新登录或重启生效
```

### 3. 测试串口连接

```bash
# 配置串口参数
sudo stty -F /dev/ttyUSB0 115200 raw -echo

# 读取数据测试
timeout 3 cat /dev/ttyUSB0

# 应该能看到 JSON 格式的 IMU 数据
```

---

## Docker 启动命令对比

### 开发环境 (WSL2)

```bash
# 使用开发配置启动
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch imu_serial_publisher imu_dev_wsl.launch.py
```

### 生产环境 (树莓派)

```bash
# 方式1: 使用 docker-compose
cd ~/robot_project
docker-compose up -d

# 方式2: 直接运行 Docker
docker run -d \
  --name robot_controller \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --network=host \
  -v $(pwd)/workspace:/workspace \
  ros2-humble-dev:latest \
  bash -c "source /opt/ros/humble/setup.bash && ros2 launch imu_serial_publisher imu_raspberry_pi.launch.py"

# 方式3: 在容器内手动启动
docker exec -it robot_controller bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch imu_serial_publisher imu_raspberry_pi.launch.py
```

---

## 配置文件选择

### 开发时 (本地 Windows + WSL2)

```bash
# 使用 WSL 配置
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py

# 或手动指定配置文件
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args \
  --params-file /workspace/robot_ws/src/imu_serial_publisher/config/imu_params_dev_wsl.yaml
```

---

## IMU 数据健康检查

### 快速检查命令

```bash
# 1. 检查主题是否发布
ros2 topic list | grep imu

# 2. 查看一条消息
ros2 topic echo /imu/data --once

# 3. 检查发布频率
ros2 topic hz /imu/data

# 4. 运行完整健康检查 (推荐)
cd /workspace/robot_ws
python3 scripts/imu_health_check.py
```

### 健康检查工具说明

**功能**: 自动收集 100 个 IMU 数据样本,分析:
- ✓ 采样率是否正常
- ✓ 加速度计数据质量(重力加速度、噪声水平)
- ✓ 陀螺仪零偏和稳定性
- ✓ 姿态稳定性和水平度

**使用方法**:
```bash
# 1. 将 IMU 静止水平放置在桌面上
# 2. 运行检查工具
python3 /workspace/robot_ws/scripts/imu_health_check.py
# 3. 等待约 15 秒完成数据收集
# 4. 查看分析报告
```

**常见问题诊断**:

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| **Z轴加速度异常 (946 m/s² 误差)** | 加速度单位错误 | IMU 输出的是 **mg (毫重力)**, 需要转换为 m/s²: `value * 9.81 / 1000` |
| **角速度持续变化** | 陀螺仪零偏未校准 | 执行陀螺仪校准程序 |
| **采样率偏低 (7 Hz)** | 串口读取效率低 | 已优化为缓冲读取,但受限于定时器频率 |
| **加速度噪声大** | 桌面震动 / IMU 未固定 | 使用减震垫 / 固定 IMU |
| **姿态抖动** | 传感器噪声 / 融合算法 | 检查 IMU 固件配置 |

### 部署到树莓派

```bash
# 使用树莓派配置
ros2 launch imu_serial_publisher imu_raspberry_pi.launch.py

# 或手动指定配置文件
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args \
  --params-file /workspace/robot_ws/src/imu_serial_publisher/config/imu_params_raspberry_pi.yaml
```

---

## 常见问题

### 问题1: 树莓派上找不到串口设备

**解决:**
```bash
# 检查 USB 设备是否识别
lsusb

# 查看内核日志
dmesg | grep tty

# 手动加载驱动
sudo modprobe usbserial
sudo modprobe cp210x  # CP210x 芯片
sudo modprobe ch341   # CH340 芯片
```

### 问题2: Docker 容器内无权限访问串口

**解决:**
```bash
# 方法1: 在宿主机修改设备权限
sudo chmod 666 /dev/ttyUSB0

# 方法2: 使用 --privileged
docker run --privileged ...

# 方法3: 添加用户到 dialout 组
docker exec robot_controller usermod -a -G dialout root
```

### 问题3: 设备号变化 (/dev/ttyUSB0 变成 /dev/ttyUSB1)

**解决:**
```bash
# 使用固定的设备 ID
serial_port: "/dev/serial/by-id/usb-Silicon_Labs_CP2102_..."

# 或创建 udev 规则
sudo nano /etc/udev/rules.d/99-usb-serial.rules

# 添加规则:
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyIMU"

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 之后可以使用 /dev/ttyIMU
```

---

## 快速部署检查清单

### 树莓派首次部署

- [ ] 树莓派系统已安装 Docker
- [ ] 用户已添加到 docker 和 dialout 组
- [ ] USB 串口已连接,能在 `/dev/ttyUSB*` 看到设备
- [ ] 已测试串口数据读取正常
- [ ] Docker 镜像已构建或传输到树莓派
- [ ] docker-compose.yml 已配置设备映射
- [ ] 配置文件使用 `imu_params_raspberry_pi.yaml`
- [ ] launch 文件使用 `imu_raspberry_pi.launch.py`

### 开发环境 (WSL2)

- [ ] Windows 已安装 usbipd-win
- [ ] WSL2 已安装 USB/IP 工具
- [ ] USB 设备已通过 usbipd attach 连接
- [ ] Docker 容器能看到 `/dev/ttyUSB0`
- [ ] 配置文件使用 `imu_params_dev_wsl.yaml`
- [ ] launch 文件使用 `imu_dev_wsl.launch.py`

---

**更新日期:** 2025-12-05
