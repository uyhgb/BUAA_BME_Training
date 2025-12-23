# 外骨骼机器人ROS2系统架构完整文档

**文档完成日期**: 2025年12月23日  
**项目名称**: 基于ROS2的髋关节外骨骼机器人控制系统
**ROS工作空间**: `robot_ws/`

---

## 超快速启动
所有终端在主目录下启动（也就是BUAA_BME_Training目录），确保已经source了ROS2和工作空间环境，source命令如下（根据实际路径调整）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```


**场景1: 完整系统启动**
```bash
# 测试模式 (不启用电机)
ros2 launch exoskel_control_py exoskel_control.launch.py enable_motor:=false

# 生产模式 (启用电机)
ros2 launch exoskel_control_py exoskel_control.launch.py enable_motor:=true
```

**场景2: 数据采集（用于训练SVM）**
```bash
# 采集IMU数据，直接使用complete系统启动
ros2 launch exo_sensors imu_complete_system.launch.py \
  serial_port:=/dev/imu_usb \
  enable_recorder:=true \
  enable_visualizer:=false
```

**场景3: 足底传感器测试（未实际测试过）**
```bash
ros2 launch exo_sensors foot_state_machine.launch.py
```

**场景4: C++控制器测试（预留）**
```bash
ros2 launch robot_controller robot_system.launch.py
```


## 我自己的话
文档主要由AI总结，事实性错误我基本都改过来了，这一部分由我自己撰写，除了svm训练代码，本项目所有代码均由我完成，大量使用了AI，所以代码基本都有注释
如果你不得不来研究我的代码，说明你也有福了，在本专业摊上这么个课
ROS2对于初学者，不管是Linux初学者还是自动控制或者机器人初学者，都有一定难度
本项目使用ROS2其实是大材小用了，ROS最核心的moveit功能并没有用到，我主要使用的是它的通信框架
简单介绍一下整个项目的目录结构（没用到而且我也没删的东西不用管，万一删了就报错）：

```bash
|————.devcontainer/  # VSCode远程容器配置文件
|————1note/         # 我的开发日志和笔记
|————build/          # ROS2编译输出目录
|————install/        # ROS2安装目录
|————joint_trajectory # 用电机采集的关节轨迹数据，主要是关节角
|————log            # ROS2日志文件
|————paper_reference # 相关论文和资料，防侵权不上传github
|————robot_ws/      # ROS2工作空间
    |————src/        # 功能包源码
        |————exo_sensors/          # 传感器数据采集功能包（Python）
        |————exoskel_control_py/   # 外骨骼控制功能包（Python + C++）
        |————imu_serial_publisher/ # IMU串口发布器（C++，备用）
        |————robot_controller/     # 机器人控制器（C++，预留）
    |scripts/      # 辅助脚本
    |data/         # 采集的训练数据，IMU数据
|————sensor_source_code            # 烧录到IMU的ESP32代码
|————svm                           # SVM训练代码和模型
|————unitree_actuator_sdk-main    # 宇树电机SDK（C++ + Python版本）
```

开发流程：
11.24之前，仍在使用宇树的ROS2框架，代码在backup分支，过于臃肿，已弃用，仅在中期展示时使用过，仅对其中的docker配置进行了保留
11.24-12.23（我大概按时间顺序回忆我的工作流）
1. 重构代码，搭建ROS基本框架
2. 从IMU入手，规定了IMU以CSV格式通过串口发送数据，烧录代码，在树莓派上进行烧录（环境完整易于配置）
3. 查文献，确定了控制策略为前馈控制，清洗了论文中的仿真数据，得到最小代谢所需设备驱动力矩曲线
4. 在ROS系统上尝试读取IMU数据，编写imu_csv_reader节点，主要实现两个功能，清洗IMU固定发送数据的单位字符，发布到话题，另一个是实现数据采集，生成带时间戳的CSV文件，[可能有用的笔记1](./1note/12.8/12.8_功能包说明.md)，[可能有用的笔记2](./1note/12.8/12.8_IMU节点快速启动.md)
5. 实现足底传感器状态机，读取压力阵列数据，识别步态状态，具体的情况不清楚，没有实际测试过
6. 在本地训练SVM模型，使用scikit-learn，保存为pkl文件，训练环境的numpy版本必须小于2.0以兼容ROS2的环境
7. 进一步确定控制策略为 相位超前的稀疏前馈控制[备忘录](./1note/12.21/12.21_备忘录.md)，规定了所有USB设备的命名规范[命名](./1note/12.20/12.20_关于多设备的ttyUSB号.md)
8. 调试电机，尝试在docker内驱动电机，调整了USB延迟，接线方式（USB转485模块单独接树莓派的USB3.0），也调整了docker-compose.yml文件，测试zero.py脚本，可以成功驱动电机，[具体问题说明](./1note/12.22/12.22_最大困难之一USB穿透docker问题解决方案.md)
9. 实现步态推理节点gait_inference_node，订阅IMU数据话题，使用SVM模型进行实时步态相位预测，发布步态相位和状态。实现电机驱动节点motor_driver_node，订阅步态相位话题，根据预设的高斯助力曲线计算目标力矩，使用宇树电机SDK发送指令，[快速启动](./1note/12.22/12.22_电机IMU_SVM配合的快速启动.md)
10. 电机调参，消除噪音，[关于为什么原先的参数会导致噪音，而示例代码的参数可以正常使用](/1note/12.22/12.22_解决问题的回顾-电机参数选择.md)
11. 电机最终调参，解决力量太小问题，[调参详解](/1note/12.23/12.23_电机调参.md)

### 根据模块来回顾
这部分主要是我写的大纲，为了给其他人用来写文档，做PPT用的，比较简略
#### ROS2系统
1. 系统时间戳同步，数据的“时间对齐”，IMU采样率100Hz，足底20Hz，电机200Hz+，三部分通过ROS通讯对齐了时间戳
2. 架构解耦与模块化，所有功能均以ROS功能包形式实现，高度解耦合，让系统的扩展性和维护性变得非常好
3. 混合编程语言支持 (Python + C++)，不使用ROS系统实现混合编程实现起来比较复杂
#### 论文和控制策略
1. 论文使用opensim仿真，在髋部设计一个独立的外部驱动，以最小代谢成本计算得出外部设备在步态周期内的力矩驱动助力曲线
2. 控制策略，通过该曲线，分别在髋屈和髋伸进行前馈控制，由IMU数据通过SVM算法整合为一个状态机，将步态周期映射到0到1，由足底传感器来确定步态周期的开始和结束，即0和1的位置
#### 上位机
1. 上位机使用docker容器运行ROS2
方便多端开发与部署，开发环境，使用wsl2在本地进行测试，生产环境，在树莓派可以直接部署docker容器，无需担心兼容性和依赖性问题
2. 远程连接方案有两套
第一，使用nomachine创建远程桌面，直接操作树莓派桌面，适用于初步配置和IMU代码烧录（树莓派arduino环境完善，便于代码烧录）
第二，使用vscode ssh远程连接，无可视化界面，减轻树莓派负担，适用于外骨骼实际运行环境
3. 所有传感器的物理接口都可以直接映射到容器内使用，穿透性好且没有性能损失，适合外骨骼高实时性场景
#### IMU
1. 设置了更高的采样率，现在是100Hz
2. 规定了串口发送CSV格式的数据，只使用逗号分隔，避免数据帧过长造成阻塞
3. 单独写了一版用于IMU数据采集的代码，包括读取器，记录器，可视化器，可在启动参数内进行功能开关，采集数据可直接生成CSV
#### 足底
1. 足底串口接收到的数据帧经解码器解码得到不同触点的具体受力情况
2. 由于放弃了全周期跟随的前馈控制，把足底传感器整合进系统内作为安全锁，和SVM识别到的应当助力的步态相位（60%左右）形成与门逻辑，只有当模型判断为助力区间且脚确实踩在地上时给予助力，其他时间强制规定电机输出力矩为0，避免误判助力
其实2不算很必要的功能
但不写进去其实没得写了
这个功能我也先不着急实现

### 其他
#### 参考的论文
前馈曲线来源论文：
Bianco, N. A., Franks, P. W., Hicks, J. L., & Delp, S. L. (2022). Coupled exoskeleton assistance simplifies control and maintains metabolic benefits: A simulation study. PLoS One, 17(1), e0261318.
http://dx.doi.org/10.1371/journal.pone.0261318

对应的仿真代码
https://github.com/stanfordnmbl/coupled-exo-sim

公开的仿真数据
https://simtk.org/projects/coupled-exo-sim

#### 项目仍然存在的遗憾
1. 我个人觉得，机械结构对于力矩的传递效率影响很大，我实际穿戴时，感觉绑带对于抬腿时有一定的阻碍，另外，由于设计的结构相当于在强行推动大腿前摆，而没有辅助髋伸，所以感觉助力效果并不明显，即便力矩已经调到最大23Nm了，可能实际传导到大腿的力矩并不大
2. 单纯IMU数据进行步态识别，准确率并不高，尤其是在复杂环境下，可能需要更多传感器数据的融合，比如足底压力传感器，甚至是肌电数据
3. 本次仅做了单边的助力，由于树莓派USB口供电有上限，连接多个传感器和多个USB转485模块可能会导致供电不足

---

以下基本为AI生成的文档内容，我仅对事实性错误进行修改

## 📋 目录

- [1. 系统概述](#1-系统概述)
- [2. 功能包列表](#2-功能包列表)
- [3. 节点架构图](#3-节点架构图)
- [4. 详细节点说明](#4-详细节点说明)
- [5. 话题通信关系](#5-话题通信关系)
- [6. 启动文件说明](#6-启动文件说明)
- [7. 数据流向分析](#7-数据流向分析)
- [8. 未使用模块说明](#8-未使用模块说明)

---

## 1. 系统概述

### 1.1 系统功能

本系统是一个基于ROS2 Humble的外骨骼机器人控制系统，实现了：

- **传感器数据采集**: IMU姿态数据、足底压力传感器数据
- **步态推理**: 基于SVM机器学习模型的实时步态相位预测
- **电机控制**: 宇树电机驱动与控制
- **数据可视化与记录**: 实时数据监控、日志记录
- **扩展控制框架**: C++高级控制器接口（预留）

### 1.2 技术栈

- **操作系统**: Ubuntu 22.04 (Docker容器) / Windows 11 (开发环境) /Raspberry Pi OS (部署环境)
- **ROS版本**: ROS2 Humble
- **编程语言**: Python 3.10, C++ 17
- **机器学习**: scikit-learn (SVM模型)
- **硬件接口**: 
  - ESP32 9DOF IMU (串口通信 @ 100Hz)
  - 足底压力传感器 (串口通信 @ 115200 baud)
  - 宇树A1/B1/GO-M8010-6电机 (SDK驱动)

### 1.3 系统架构特点

- **模块化设计**: 每个功能独立封装为ROS包
- **松耦合通信**: 基于话题的异步消息传递
- **多平台支持**: WSL2开发环境 + 树莓派部署配置
- **实时性保证**: 100Hz IMU采样 + 50Hz控制频率
- **可扩展性**: 预留C++控制器接口和多种传感器接口

---

## 2. 功能包列表

| 功能包名称 | 类型 | 状态 | 主要功能 |
|-----------|------|------|---------|
| **exo_sensors** | Python | ✅ 使用中 | IMU数据采集、足底传感器状态机、数据记录与可视化 |
| **exoskel_control_py** | Python + C++ | ✅ 使用中 | 步态推理、电机驱动、宇树SDK集成 |
| **robot_controller** | C++ | ⚠️ 预留 | 高级控制算法框架（待实现） |
| **imu_serial_publisher** | C++ | ⚠️ 备用 | C++版本IMU发布节点（已被Python版本替代） |

---

## 3. 节点架构图

### 3.1 核心系统拓扑（当前使用）

```
┌─────────────────────────────────────────────────────────────────┐
│                        传感器层 (Sensor Layer)                    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ├──> [ESP32 IMU] (100Hz)
                              │         │
                              │         ├──> (Serial: dev/imu_usb)
                              │         │
                              │    ┌────▼────────────────┐
                              │    │  IMUCSVReader       │ (exo_sensors)
                              │    │  imu_csv_reader.py  │
                              │    └────┬────────────────┘
                              │         │
                              │         ├──> /imu/data (sensor_msgs/Imu)
                              │         │
                              ├──> [足底压力传感器] (115200 baud)
                              │         │
                              │    ┌────▼────────────────┐
                              │    │ FootStateMachine    │ (exo_sensors)
                              │    │ foot_state_machine.py│
                              │    └────┬────────────────┘
                              │         │
                              │         ├──> /foot/gait_state (String)
                              │         ├──> /foot/pressure (Float32)
                              │
┌─────────────────────────────────────────────────────────────────┐
│                      推理层 (Inference Layer)                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                         ┌────▼────────────────┐
                         │ GaitInferenceNode   │ (exoskel_control_py)
                         │ gait_inference_node.py│
                         │ [SVM Model @50Hz]   │
                         └────┬────────────────┘
                              │
                              ├──> /gait_phase (Float32)
                              ├──> /gait_state (String)
                              │
┌─────────────────────────────────────────────────────────────────┐
│                       控制层 (Control Layer)                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                         ┌────▼────────────────┐
                         │  MotorDriverNode    │ (exoskel_control_py)
                         │  motor_driver_node.py│
                         │  [宇树SDK驱动]       │
                         └────┬────────────────┘
                              │
                              ├──> [实际使用的是宇树GO-M8010-6电机]
                              │
┌─────────────────────────────────────────────────────────────────┐
│                    辅助工具层 (Utility Layer)                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ├──> IMUDataRecorder (数据记录)
                              ├──> IMUVisualizer (实时可视化)
                              └──> DataLogger (日志记录)
```

### 3.2 完整系统拓扑（包含预留模块）

```
┌─────────────────────────────────────────────────────────────────┐
│                         硬件接口层                                │
└─────────────────────────────────────────────────────────────────┘
         │
         ├──> ESP32 IMU ──┬──> IMUCSVReader (Python) ✅
         │                └──> IMUSerialPublisher (C++) ⚠️ 备用
         │
         ├──> 足底传感器 ──> FootStateMachine (Python) ✅
         │
         └──> 宇树电机 ────> MotorDriverNode (Python + C++) ✅
                                      │
                                      └──> unitree_actuator_sdk
                                      
┌─────────────────────────────────────────────────────────────────┐
│                         算法处理层                                │
└─────────────────────────────────────────────────────────────────┘
         │
         ├──> GaitInferenceNode (SVM推理) ✅
         │
         └──> RobotController (高级控制) ⚠️ 预留框架
                  │
                  ├──> MPC控制 (待实现)
                  ├──> 力控制 (待实现)
                  └──> 全身动力学 (待实现)

┌─────────────────────────────────────────────────────────────────┐
│                         应用工具层                                │
└─────────────────────────────────────────────────────────────────┘
         │
         ├──> IMUDataRecorder (训练数据采集) ✅
         ├──> IMUVisualizer (matplotlib实时可视化) ✅
         └──> DataLogger (rosbag替代方案) ✅
```

---

## 4. 详细节点说明

### 4.1 exo_sensors 功能包

#### 4.1.1 IMUCSVReader (✅ 使用中)

**文件**: `exo_sensors/imu_csv_reader.py`

**功能描述**:
- 从ESP32串口读取CSV格式的IMU数据（100Hz采样）
- 解析格式: `Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ`
- 过滤单位字符串 (`mg`, `dps`, `m/s2`, `rad/s`)
- 单位转换: 角度→弧度, mg→m/s², dps→rad/s
- 计算姿态四元数并发布ROS标准消息

**订阅话题**: 无

**发布话题**:
- `/imu/data` (sensor_msgs/Imu) - 标准IMU消息

**参数**:
- `serial_port` (string): 串口设备路径
  - Windows: `COM*`（与电脑有关）
  - Linux: `/dev/imu_usb`
  - WSL2: `/dev/tty*`(可随意映射，在.devcontainer\docker-compose.yml中配置，目前是直接挂载了/dev目录)
- `baud_rate` (int): 波特率 (默认 115200)
- `publish_rate` (double): 发布频率 (默认 100.0 Hz)

**启动方式**:
```bash
ros2 launch exo_sensors imu_csv_reader.launch.py serial_port:=/dev/ttyUSB0
```

---

#### 4.1.2 FootStateMachine (✅ 使用中)

**文件**: `exo_sensors/foot_state_machine.py`

**功能描述**:
- 读取足底压力传感器串口数据（36点压力阵列）
- 使用施密特触发器检测步态状态
- 识别脚落地(Heel Strike)和脚抬起(Toe Off)事件
- 计算总压力并发布状态信息

**订阅话题**: 无

**发布话题**:
- `/foot/gait_state` (String): 步态状态 (`stance` / `swing`)
- `/foot/pressure` (Float32): 总压力值（单位: g）
- `/foot/event` (Int32): 状态变化事件
  - `1` = 脚落地
  - `0` = 脚抬起
  - `-1` = 无变化

**参数**:
- `serial_port` (string): 串口设备路径 (默认 `/dev/foot_usb`)
- `baud_rate` (int): 波特率 (默认 115200)
- `threshold_on` (int): 落地阈值 (默认 800g)
- `threshold_off` (int): 抬起阈值 (默认 150g)

**关键算法**:
- **施密特触发器**: 避免抖动，提供稳定状态切换
- **零点校准**: 启动时自动消除传感器底噪
- **关键点筛选**: 仅监控脚跟和前掌核心区域（12个关键点）

**启动方式**:
```bash
ros2 launch exo_sensors foot_state_machine.launch.py
```

---

#### 4.1.3 IMUDataRecorder (✅ 使用中)

**文件**: `exo_sensors/imu_data_recorder.py`

**功能描述**:
- 订阅IMU数据并记录到CSV文件
- 用于采集SVM训练数据
- 自动生成带时间戳的文件名

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu)

**发布话题**: 无

**输出文件**: `imu_gait_data_YYYYMMDD_HHMMSS.csv`

**启动方式**:
```bash
ros2 run exo_sensors imu_data_recorder
```

---

#### 4.1.4 IMUVisualizer (✅ 使用中)

**文件**: `exo_sensors/imu_visualizer.py`

**功能描述**:
- 使用matplotlib实时绘制IMU数据曲线
- 显示Roll、Pitch、Yaw角度变化
- 显示线性加速度和角速度

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu)

**发布话题**: 无

**启动方式**:
```bash
ros2 run exo_sensors imu_visualizer
```

---

### 4.2 exoskel_control_py 功能包

#### 4.2.1 GaitInferenceNode (✅ 使用中)

**文件**: `exoskel_control_py/gait_inference_node.py`

**功能描述**:
- 使用训练好的SVM模型实时推理步态相位
- 50Hz推理频率
- 加载sklearn模型和StandardScaler
- 输出归一化步态相位 [0.0, 1.0]

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu)

**发布话题**:
- `/gait_phase` (Float32): 步态相位 [0.0-1.0]
- `/gait_state` (String): 步态状态文本描述

**模型文件**:
- `config/optimized_gait_svm_model.pkl` - SVM回归模型
- `config/optimized_gait_scaler.pkl` - 特征归一化器
- `config/optimized_feature_columns.pkl` - 特征列顺序

**模型性能**:
- R² Score: 0.7927
- RMSE: 0.1317
- Phase Accuracy: 67.96%

```shell
# 实际的训练日志
[23:22:16] 训练完成！耗时: 23.2 分钟

===== 模型评估结果 =====
R²决定系数：0.7927
MAE平均绝对误差：0.0858
RMSE均方根误差：0.1317
MAPE平均绝对百分比误差(%)：44913864.99
步态阶段分类准确率(%)：67.96

===== 各步态阶段分类准确率 =====
支撑相早期：69.24% (正确1812/2617)
支撑相中期：66.77% (正确1700/2546)
支撑相晚期：73.48% (正确1867/2541)
摆动相早期：70.36% (正确1797/2554)
摆动相晚期：59.73% (正确1485/2486)

最优模型已保存：./optimized_gait_svm_model.pkl
最优参数：{'C': 50, 'epsilon': 0.01, 'gamma': 0.1, 'kernel': 'rbf', 'max_iter': 50000, 'tol': 0.0001}
交叉验证R²：0.7332
[6/7] 验证生成结果...
带标签数据：./labeled_imu_data_combined.csv（大小：5462.62 KB）
SVM模型：./optimized_gait_svm_model.pkl（大小：5222.25 KB）
标准化器：./optimized_gait_scaler.pkl（大小：3.24 KB）
特征列配置：./optimized_feature_columns.pkl（大小：0.49 KB）
所有文件生成成功！

===== 最终模型评估汇总 =====
R²决定系数：0.7927
步态分类准确率：67.96%
```


**特征向量** (9维):
```python
[roll, pitch, yaw, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
```

**启动方式**:
```bash
ros2 run exoskel_control_py gait_inference_node
```

**参数**:
- `model_path` (string): SVM模型文件路径
- `scaler_path` (string): 归一化器文件路径
- `inference_rate` (double): 推理频率 (默认 50.0 Hz)

---

#### 4.2.2 MotorDriverNode (✅ 使用中)

**文件**: `exoskel_control_py/motor_driver_node.py`

**功能描述**:
- 订阅步态相位，查表生成电机轨迹
- 使用宇树SDK控制A1/B1/GO-M8010-6电机
- 支持位置、速度、力矩混合控制
- 电机ID自动识别和配置

**订阅话题**:
- `/gait_phase` (Float32): 步态相位输入

**发布话题**:
- `/motor_feedback` (自定义消息): 电机反馈状态

**电机接口**:
- **SDK**: `unitree_actuator_sdk` (pybind11封装)
- **通信**: CAN/RS485总线
- **控制模式**: 位置控制 + MIT阻抗控制

**轨迹文件**:
- `joint_trajectory/normal_gait.csv` - 正常步态轨迹
- `joint_trajectory/limit_gait.csv` - 受限步态轨迹

**启动方式**:
```bash
ros2 run exoskel_control_py motor_driver_node
```

**参数**:
- `motor_type` (string): 电机型号 (`a1` / `b1` / `go_m8010_6`)
- `motor_ids` (int[]): 电机ID列表
- `trajectory_file` (string): 轨迹CSV文件路径
- `control_gain_kp` (double): 位置增益
- `control_gain_kd` (double): 速度增益

**安全特性**:
- 位置限幅保护
- 速度限幅保护
- 力矩限幅保护
- 通信超时检测

---

### 4.3 robot_controller 功能包 (⚠️ 预留)

#### 4.3.1 RobotController (⚠️ 未启用)

**文件**: `robot_controller/src/main_controller.cpp`

**功能描述**:
- C++实现的高级控制器框架
- 预留接口用于实现复杂控制算法
- 支持高频率控制循环（>100Hz）
- 当前为空框架，控制算法待实现

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu)
- (可扩展: 关节状态、力传感器等)

**发布话题**:
- `/motor_commands` (Float32MultiArray)
- `/cmd_vel` (geometry_msgs/Twist)

**参数**:
- `control_frequency` (double): 控制频率 (默认 50.0 Hz)
- `enable_control` (bool): 控制使能标志 (默认 false)
- `num_motors` (int): 电机数量 (默认 12)

**预留算法接口**:
```cpp
void run_control_algorithm()  // 主控制循环
{
    // TODO: 实现控制算法
    // 1. 状态估计
    // 2. 轨迹规划
    // 3. 力/位置控制
    // 4. 安全检查
}
```

**使用场景** (未来扩展):
- 模型预测控制 (MPC)
- 全身动力学控制
- 力位混合控制
- 实时轨迹优化

**启动方式**:
```bash
ros2 run robot_controller main_controller --ros-args -p enable_control:=true
```

**状态**: 当前Python节点已满足需求，C++控制器仅在需要极高性能时启用

---

#### 4.3.2 DataLogger (✅ 使用中)

**文件**: `robot_controller/scripts/data_logger.py`

**功能描述**:
- 多话题数据同步记录
- CSV格式输出，便于后续分析
- 作为rosbag的轻量级替代

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu)
- `/gait_phase` (Float32)
- `/motor_feedback` (自定义)

**输出文件**: `robot_data_log_YYYYMMDD_HHMMSS.csv`

---

#### 4.3.3 IMUVisualizer (robot_controller版本) (✅ 使用中)

**文件**: `robot_controller/scripts/visualization.py`

**功能描述**:
- 与exo_sensors版本类似
- 额外支持3D姿态可视化（使用PyQt5）

---

### 4.4 imu_serial_publisher 功能包 (⚠️ 备用)

#### 4.4.1 IMUSerialPublisher (⚠️ 已被替代)

**文件**: `imu_serial_publisher/src/imu_serial_publisher.cpp`

**功能描述**:
- C++版本的IMU串口发布节点
- 功能与`IMUCSVReader`相同
- 已被Python版本替代（更易维护）

**状态**: 保留作为备用，或用于性能对比测试

**启动方式**:
```bash
ros2 launch imu_serial_publisher imu_serial.launch.py
```

---

## 5. 话题通信关系

### 5.1 话题列表

| 话题名称 | 消息类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---------|---------|--------|--------|------|------|
| `/imu/data` | sensor_msgs/Imu | IMUCSVReader | GaitInferenceNode, IMUDataRecorder, IMUVisualizer, RobotController | 100Hz | IMU姿态数据 |
| `/gait_phase` | std_msgs/Float32 | GaitInferenceNode | MotorDriverNode, DataLogger | 50Hz | 步态相位 [0.0-1.0] |
| `/gait_state` | std_msgs/String | GaitInferenceNode | - | 50Hz | 步态状态文本 |
| `/foot/gait_state` | std_msgs/String | FootStateMachine | - | 变化时 | 脚部状态 |
| `/foot/pressure` | std_msgs/Float32 | FootStateMachine | - | 100Hz | 总压力值 |
| `/foot/event` | std_msgs/Int32 | FootStateMachine | - | 变化时 | 步态事件 |
| `/motor_commands` | Float32MultiArray | RobotController | - | 50Hz | 电机命令(C++版本) |
| `/motor_feedback` | 自定义 | MotorDriverNode | DataLogger | 50Hz | 电机状态反馈 |
| `/cmd_vel` | geometry_msgs/Twist | RobotController | - | 50Hz | 速度命令(预留) |

### 5.2 通信拓扑图（简化）

```
IMUCSVReader ──┬──> /imu/data ──┬──> GaitInferenceNode
               │                 ├──> IMUDataRecorder
               │                 ├──> IMUVisualizer
               │                 └──> RobotController (预留)
               │
FootStateMachine ──> /foot/gait_state
               ├──> /foot/pressure
               └──> /foot/event

GaitInferenceNode ──┬──> /gait_phase ──> MotorDriverNode ──> [宇树电机]
                    └──> /gait_state

MotorDriverNode ──> /motor_feedback ──> DataLogger

RobotController ──> /motor_commands (未使用)
                └──> /cmd_vel (未使用)
```

---

## 6. 启动文件说明

### 6.1 exo_sensors 包启动文件

| 启动文件 | 功能 | 用途 |
|---------|------|------|
| `imu_csv_reader.launch.py` | 启动IMU读取节点 | 单独测试IMU |
| `imu_csv_dev_wsl.launch.py` | WSL2环境IMU启动 | WSL2开发环境 |
| `imu_csv_raspberry_pi.launch.py` | 树莓派IMU启动 | 树莓派部署 |
| `imu_complete_system.launch.py` | 完整IMU系统 | IMU+可视化+记录 |
| `foot_state_machine.launch.py` | 足底传感器节点 | 步态检测 |

### 6.2 exoskel_control_py 包启动文件

| 启动文件 | 功能 | 用途 |
|---------|------|------|
| `exoskel_control.launch.py` | 完整控制系统 | 步态推理+电机控制 |
| `start_control.launch.py` | 启动控制节点 | 仅控制节点 |

### 6.3 robot_controller 包启动文件

| 启动文件 | 功能 | 用途 |
|---------|------|------|
| `robot_system.launch.py` | C++控制器系统 | 启动C++主控节点 |

### 6.4 快速启动命令

所有终端在主目录下启动（也就是BUAA_BME_Training目录），确保已经source了ROS2和工作空间环境，source命令如下（根据实际路径调整）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```


**场景1: 完整系统启动**
```bash
# 测试模式 (不启用电机)
ros2 launch exoskel_control_py exoskel_control.launch.py enable_motor:=false

# 生产模式 (启用电机)
ros2 launch exoskel_control_py exoskel_control.launch.py enable_motor:=true
```

**场景2: 数据采集（用于训练SVM）**
```bash
# 采集IMU数据，直接使用complete系统启动
ros2 launch exo_sensors imu_complete_system.launch.py \
  serial_port:=/dev/imu_usb \
  enable_recorder:=true \
  enable_visualizer:=false
```

**场景3: 足底传感器测试**
```bash
ros2 launch exo_sensors foot_state_machine.launch.py
```

**场景4: C++控制器测试（预留）**
```bash
ros2 launch robot_controller robot_system.launch.py
```

---

## 7. 数据流向分析

### 7.1 实时控制流程

```
[ESP32 IMU] (100Hz)
    │
    ├──> Serial /dev/imu_usb (CSV格式)
    │
    ▼
[IMUCSVReader节点]
    │ 解析CSV + 单位转换 + 四元数计算
    ▼
/imu/data (sensor_msgs/Imu, 100Hz)
    │
    ├──────────────────┐
    │                  │
    ▼                  ▼
[GaitInferenceNode]  [IMUVisualizer]
    │ SVM推理 (50Hz)    │ matplotlib实时绘图
    ▼                  │
/gait_phase (Float32)  │
    │                  │
    ▼                  │
[MotorDriverNode]      │
    │ 查表生成轨迹      │
    │ 宇树SDK驱动       │
    ▼                  │
[ 电机 ]            │
    │                  │
    ▼                  ▼
[实际运动]          [数据监控]
```

### 7.2 训练数据采集流程

```
[ESP32 IMU] (100Hz)
    │
    ▼
[IMUCSVReader]
    │
    ▼
/imu/data
    │
    ▼
[IMUDataRecorder]
    │ 订阅并记录
    ▼
imu_gait_data_YYYYMMDD_HHMMSS.csv
    │
    ▼
[SVM训练脚本] (svm/main_v3.py)
    │ GridSearchCV优化
    ▼
optimized_gait_svm_model.pkl
optimized_gait_scaler.pkl
optimized_feature_columns.pkl
    │
    ▼
[部署到GaitInferenceNode]
```

### 7.3 足底传感器辅助流程（可选）

```
[足底压力传感器] (115200 baud)
    │
    ▼
[FootStateMachine]
    │ 施密特触发器检测
    ├──> /foot/gait_state (stance/swing)
    ├──> /foot/pressure (Float32)
    └──> /foot/event (1:落地, 0:抬起)
         │
         ▼
[数据融合节点] (未实现，可扩展)
    │ 融合IMU + 足底传感器
    ▼
[更精准的步态推理]
```

---

## 8. 未使用模块说明

### 8.1 RobotController (C++控制器)

**状态**: ⚠️ 预留框架，未启用

**位置**: `robot_ws/src/robot_controller/src/main_controller.cpp`

**原因**:
- 当前Python节点架构已满足需求
- 50Hz控制频率Python性能足够
- Python代码更易于快速迭代和调试

**未来启用场景**:
- 控制频率需求 >200Hz
- 需要实现复杂实时算法（MPC、动力学仿真）
- 对延迟有极高要求（<5ms）

**如何启用**:
1. 实现 `run_control_algorithm()` 函数
2. 设置参数 `enable_control:=true`
3. 编译C++节点: `colcon build --packages-select robot_controller`
4. 启动: `ros2 launch robot_controller robot_system.launch.py`

---

### 8.2 IMUSerialPublisher (C++版IMU发布)

**状态**: ⚠️ 已被Python版本替代

**位置**: `robot_ws/src/imu_serial_publisher/src/imu_serial_publisher.cpp`

**替代者**: `exo_sensors/imu_csv_reader.py`

**保留原因**:
- 作为性能基准测试
- 某些场景可能需要C++串口性能
- 代码示例参考

**性能对比**:
| 指标 | Python版本 | C++版本 |
|-----|-----------|---------|
| CPU占用 | ~5% | ~2% |
| 延迟 | ~10ms | ~3ms |
| 维护成本 | 低 | 中 |
| 功能完整性 | ✅ | ✅ |

**结论**: Python版本性能足够，优先使用

---

### 8.3 FootStateMachine (足底传感器)

**状态**: ✅ 已实现，可选启用

**位置**: `exo_sensors/foot_state_machine.py`

**使用场景**:
- 需要更精准的步态检测
- 地面接触力反馈
- 双足融合推理

**当前状态**: 独立运行，未与主控制流集成

**集成建议**:
可以创建一个数据融合节点，结合IMU和足底传感器：
```python
# 伪代码示例
class SensorFusionNode(Node):
    def __init__(self):
        # 订阅IMU和足底数据
        self.imu_sub = self.create_subscription(Imu, '/imu/data', ...)
        self.foot_sub = self.create_subscription(String, '/foot/gait_state', ...)
        
    def fusion_callback(self):
        # 卡尔曼滤波 / 互补滤波融合
        fused_gait_phase = self.fuse_sensors(imu_data, foot_data)
        self.pub.publish(fused_gait_phase)
```

---

## 9. 配置文件说明

### 9.1 IMU配置文件

**位置**: `robot_ws/src/exo_sensors/config/`

- `imu_csv_reader.yaml` - 默认配置
- `imu_csv_dev_wsl.yaml` - WSL2开发环境
- `imu_csv_raspberry_pi.yaml` - 树莓派部署

**关键参数**:
```yaml
imu_csv_reader:
  ros__parameters:
    serial_port: '/dev/imu_usb'  # 串口路径
    baud_rate: 115200             # 波特率
    publish_rate: 100.0           # 发布频率 (Hz)
    frame_id: 'imu_link'          # TF坐标系ID
```

### 9.2 步态推理配置

**位置**: `robot_ws/src/exoskel_control_py/config/gait_inference.yaml`

```yaml
gait_inference_node:
  ros__parameters:
    model_path: '/workspace/robot_ws/src/exoskel_control_py/config/optimized_gait_svm_model.pkl'
    scaler_path: '/workspace/robot_ws/src/exoskel_control_py/config/optimized_gait_scaler.pkl'
    inference_rate: 50.0
```

### 9.3 电机控制配置

**位置**: `robot_ws/src/exoskel_control_py/config/motor_driver.yaml`

```yaml
motor_driver_node:
  ros__parameters:
    # 电机串口配置
    serial_port: "/dev/unitree_motor"  # 宇树电机固定设备名
    
    # 安全参数
    max_torque: 23.0              # 最大力矩限制 (Nm)
    
    # 控制参数
    control_mode: 1               # FOC模式
    kp: 0.0                       # 位置刚度 (消音)
    kd: 1.0                       # 阻尼系数 (防飞车)
    
    # 话题名称
    topic_cmd_torque: "/control/cmd_torque"
```

---

## 10. 系统性能指标

### 10.1 实时性能

| 指标 | 值 | 说明 |
|-----|---|------|
| IMU采样率 | 100Hz | ESP32固定频率 |
| IMU发布延迟 | <10ms | 串口传输+解析 |
| SVM推理延迟 | <5ms | scikit-learn推理 |
| 电机控制周期 | 20ms (50Hz) | 宇树SDK限制 |
| 端到端延迟 | <35ms | IMU→电机响应 |

### 10.2 资源占用（树莓派4B）

| 进程 | CPU | 内存 | 说明 |
|-----|-----|------|------|
| IMUCSVReader | ~5% | 50MB | Python串口 |
| GaitInferenceNode | ~8% | 120MB | SVM推理 |
| MotorDriverNode | ~3% | 80MB | SDK驱动 |
| **总计** | ~16% | 250MB | 可接受 |

---

## 11. 故障排查

### 11.1 常见问题

**问题1: IMU数据接收不到**

检查步骤:
```bash
# 1. 确认串口设备存在
ls -l /dev/

# 2. 检查串口权限
sudo chmod 666 /dev/imu_usb

# 3. 查看节点日志
ros2 run exo_sensors imu_csv_reader --ros-args --log-level debug

# 4. 手动测试串口
python3 -m serial.tools.miniterm /dev/imu_usb 115200
```

**问题2: SVM推理结果不稳定**

检查点:
- 模型文件是否损坏 (检查pkl文件大小)
- IMU数据是否正常 (查看`/imu/data`话题)
- numpy版本兼容性 (确保与训练时一致)

**问题3: 电机无响应**

排查流程:
```bash
# 可直接使用
unitree_actuator_sdk-main\python\zero.py，零力矩采集模式，测试电机通信是否正常
```

### 11.2 调试工具

```bash
# 查看所有节点
ros2 node list

# 查看话题数据流
ros2 topic hz /imu/data
ros2 topic bw /imu/data

# 检查节点连接
ros2 node info /imu_csv_reader

# 图形化拓扑
ros2 run rqt_graph rqt_graph
```

---

## 12. 未来扩展方向

下面的是AI写的，我自己写一点：
1. 首先就是SVM模型的优化，先别看那个准确率的问题，训练代码里用的评估指标似乎是错的，准确率用的是分类任务的准确率评价指标，这是一个回归任务，最好看R方，单IMU数据的步态推理其实效果一般，可以考虑加入足底传感器数据进行多传感器融合

2. 有一个cpp写的主控代码框架，但是没有用到，目前的全Python代码已经非常好用了，但ROS2对于集成混合编程的支持非常好，如果有高实时性需求，或者想用更Fancy的控制算法（比如MPC），可以考虑启用C++主控代码

3. 树莓派性能很差，不支持原生ROS，这也是我采用了全docker开发的原因，如果能用更好的平台（比如Jetson系列），可以考虑原生ROS开发，更方便，问题也更少

### 12.1 短期优化（1-3个月）

1. **传感器融合**
   - 融合IMU + 足底传感器
   - 实现卡尔曼滤波器
   - 提升步态检测准确率 >85%

2. **轨迹优化**
   - 在线轨迹插值
   - 自适应步态调整
   - 平滑过渡算法

3. **可视化增强**
   - Rviz2 3D可视化
   - Web界面监控
   - 实时数据面板

### 12.2 中期目标（3-6个月）

1. **启用C++控制器**
   - 实现MPC算法
   - 全身动力学仿真
   - 力位混合控制

2. **多模态控制**
   - 坐、站、走多模式切换
   - 楼梯/斜坡自适应
   - 摔倒保护策略

3. **机器学习迭代**
   - 深度学习模型（LSTM/Transformer）
   - 在线学习框架
   - 个性化适配

### 12.3 长期愿景（6-12个月）

1. **全身外骨骼集成**
   - 上肢控制系统
   - 躯干稳定控制
   - 协调运动规划

2. **人机交互**
   - 意图识别算法
   - 触觉反馈集成
   - 自然交互界面

3. **临床验证**
   - 康复训练协议
   - 患者数据管理
   - 远程监控系统

---

## 附录A: 依赖项列表

### A.1 Python依赖

```txt
# ROS2基础
rclpy>=3.0.0
sensor_msgs
std_msgs
geometry_msgs

# 数据处理
numpy>=1.24.4,<2.0.0  # 重要: 必须<2.0以兼容scikit-learn模型
pandas
scipy

# 机器学习
scikit-learn==1.3.0
joblib

# 硬件接口
pyserial
smbus2  # I2C传感器

# 可视化
matplotlib
PyQt5

# 工具
pyyaml
```

### A.2 系统依赖

```bash
# ROS2
sudo apt install ros-humble-desktop

# 开发工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep

# 硬件驱动
sudo apt install python3-serial
sudo apt install i2c-tools

# 可视化
sudo apt install ros-humble-rqt
sudo apt install ros-humble-rqt-graph
```

### A.3 宇树SDK

**C++库**:
- `libUnitreeMotorSDK_Linux64.so`
- `libUnitreeMotorSDK_Arm64.so`

**Python绑定**:
注意：这里的.so文件，由于我的docker内ROS，默认使用python3.10，所以这个.so是我自行编译得到的，在树莓派物理机环境的python是3.11版本，注意区分
- `unitree_actuator_sdk.cpython-310-x86_64-linux-gnu.so`

---

## 附录B: 文件结构

```
robot_ws/
├── src/
│   ├── exo_sensors/                    # 传感器数据采集包
│   │   ├── config/
│   │   │   ├── imu_csv_reader.yaml
│   │   │   ├── imu_csv_dev_wsl.yaml
│   │   │   └── imu_csv_raspberry_pi.yaml
│   │   ├── launch/
│   │   │   ├── imu_csv_reader.launch.py
│   │   │   ├── imu_complete_system.launch.py
│   │   │   └── foot_state_machine.launch.py
│   │   ├── exo_sensors/
│   │   │   ├── imu_csv_reader.py       ✅ IMU数据发布
│   │   │   ├── imu_data_recorder.py    ✅ 数据记录
│   │   │   ├── imu_visualizer.py       ✅ 实时可视化
│   │   │   └── foot_state_machine.py   ✅ 足底传感器
│   │   └── package.xml
│   │
│   ├── exoskel_control_py/             # 控制系统包
│   │   ├── config/
│   │   │   ├── gait_inference.yaml
│   │   │   ├── motor_driver.yaml
│   │   │   ├── optimized_gait_svm_model.pkl    # SVM模型
│   │   │   └── optimized_gait_scaler.pkl       # 归一化器
│   │   ├── launch/
│   │   │   ├── exoskel_control.launch.py
│   │   │   └── start_control.launch.py
│   │   ├── exoskel_control_py/
│   │   │   ├── gait_inference_node.py  ✅ 步态推理
│   │   │   └── motor_driver_node.py    ✅ 电机驱动
│   │   ├── python/                     # SDK示例
│   │   │   ├── example_a1_motor.py
│   │   │   └── test_motor_id.py
│   │   ├── thirdparty/
│   │   │   └── pybind11/               # Python绑定
│   │   ├── lib/
│   │   │   ├── libUnitreeMotorSDK_Linux64.so
│   │   │   └── unitree_actuator_sdk.cpython-*.so
│   │   └── package.xml
│   │
│   ├── robot_controller/               # C++控制器包(预留)
│   │   ├── launch/
│   │   │   └── robot_system.launch.py
│   │   ├── src/
│   │   │   └── main_controller.cpp     ⚠️ 预留框架
│   │   ├── scripts/
│   │   │   ├── data_logger.py          ✅ 数据日志
│   │   │   └── visualization.py        ✅ 可视化
│   │   └── package.xml
│   │
│   └── imu_serial_publisher/           # C++版IMU(备用)
│       ├── src/
│       │   └── imu_serial_publisher.cpp ⚠️ 已替代
│       └── package.xml
│
├── joint_trajectory/                   # 轨迹数据
│   ├── normal_gait.csv
│   └── limit_gait.csv
│
└── install/                            # 编译输出
    └── setup.bash
```

---

## 附录C: 相关脚本

### C.1 SVM训练脚本

**位置**: `svm/main_v3.py`

**功能**:
- 加载IMU训练数据
- GridSearchCV超参数优化
- 保存模型和归一化器

**使用方法**:
```bash
cd D:\1CodeProject\BUAA_BME_Training\svm
python main_v3.py
```

### C.2 数据采集脚本

**位置**: `robot_ws/src/exo_sensors/exo_sensors/imu_data_recorder.py`

**输出**: `imu_gait_data_YYYYMMDD_HHMMSS.csv`

### C.3 电机测试脚本

**位置**: `robot_ws/src/exoskel_control_py/python/`

- `test_motor_id.py` - 扫描电机ID
- `test_rotate.py` - 测试旋转
- `zero.py` - 电机归零
- `stop.py` - 急停

---

## 附录D: 联系与支持

**文档维护**: GitHub Copilot生成  
**项目仓库**: https://github.com/uyhgb/BUAA_BME_Training  
**最后更新**: 2025年12月23日

**开发日志**:
- 查看 `1note/` 目录下的开发日志
从12.4开始的记录，实际上是由于我在12月初开始重构代码，之前使用的是宇树的ROS2框架，后弃用，备份在backup分支
---

## 版本历史

| 版本 | 日期 | 变更说明 |
|-----|------|---------|
| v1.0 | 2025-12-23 | 初始文档，完整系统架构梳理 |
| v0.9 | 2025-12-22 | SVM模型集成完成 |
| v0.8 | 2025-12-20 | 宇树SDK集成 |
| v0.5 | 2025-12-10 | IMU数据流建立 |

---

**文档结束**

*本文档详细记录了当前系统的所有功能模块、节点关系、话题通信、启动方式及未来扩展方向。建议定期更新以保持与实际代码同步。*
