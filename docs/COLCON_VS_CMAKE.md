# Colcon vs CMake 编译详解

## 🎯 核心关系

```
┌─────────────────────────────────────────┐
│           你的 C++ 代码                  │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│         CMakeLists.txt                   │
│     (定义如何编译你的代码)                │
└────────────────┬────────────────────────┘
                 │
        ┌────────┴─────────┐
        ▼                  ▼
┌──────────────┐    ┌──────────────┐
│  直接用 cmake│    │  用 colcon   │
│  (独立项目)  │    │  (ROS2项目)  │
└──────────────┘    └──────────────┘
```

## 📊 对比表格

| 特性 | CMake (直接) | Colcon (ROS2) |
|------|-------------|---------------|
| **本质** | 构建系统生成器 | ROS2工作空间管理工具 |
| **依赖关系** | 手动管理 | 自动解析 package.xml |
| **输出位置** | `./build/` | `./build/`, `./install/` |
| **多包管理** | 需要手动配置 | 自动处理多个包 |
| **环境变量** | 手动设置 | 自动生成 setup.bash |
| **ROS2集成** | 需要手动配置 | 原生支持 |

## ⚙️ Colcon 的工作流程

```bash
colcon build
    │
    ├─> 1. 读取 package.xml (ROS2包信息)
    │
    ├─> 2. 解析依赖关系
    │
    ├─> 3. 为每个包调用 cmake
    │       mkdir build/package_name
    │       cd build/package_name
    │       cmake ../../src/package_name
    │       make
    │
    ├─> 4. 安装到 install/ 目录
    │       make install
    │
    └─> 5. 生成环境脚本
            install/setup.bash
```

## 🤔 编译内容能否共用？

### ✅ 理论上可以（但不推荐）

**Colcon 编译的内容本质上就是 CMake 编译的：**

```bash
# 这两者本质相同
colcon build                    # Colcon方式
# 等价于
cd build/unitree_ros2_example
cmake ../../src
make
make install
```

### ❌ 实践中的问题

#### 问题1: 路径不一致
```bash
# Colcon 的目录结构
unitree_ros2/
├── build/
│   └── unitree_ros2_example/    # 每个包有子目录
├── install/
│   └── unitree_ros2_example/
└── src/

# 直接用 CMake
my_project/
├── build/                       # 扁平结构
└── src/
```

#### 问题2: 环境变量
```bash
# Colcon 自动生成
source install/setup.bash        # 包含所有ROS2环境

# CMake 需要手动
export LD_LIBRARY_PATH=...       # 需要手动设置
export PATH=...
```

#### 问题3: 依赖管理
```xml
<!-- package.xml (Colcon用) -->
<depend>unitree_go</depend>      <!-- Colcon自动处理 -->
<depend>rclcpp</depend>

<!-- CMake 需要手动 -->
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
```

## 🎯 推荐方案

### 方案A: 纯 ROS2 项目 → 用 Colcon ✅

**适用：** 您当前的情况（unitree_ros2）

```bash
cd /home/weeq/unitree_ros2/example
colcon build
source install/setup.bash
ros2 run unitree_ros2_example custom_sensor_reader
```

**优点：**
- ✅ 自动处理 ROS2 依赖
- ✅ 支持多包工作空间
- ✅ 标准 ROS2 工作流

### 方案B: 独立 C++ 项目 → 用 CMake

**适用：** 不依赖 ROS2 的独立程序

```bash
mkdir build && cd build
cmake ..
make
./my_program
```

**优点：**
- ✅ 更轻量
- ✅ 不依赖 ROS2 环境
- ✅ 更快的编译（单包）

### 方案C: 混合项目 → 分离管理

**如果真的需要同时支持两种方式：**

```
my_project/
├── ros2_ws/              # ROS2 工作空间
│   ├── src/
│   │   └── my_ros_package/
│   │       ├── CMakeLists.txt    # ROS2 版本
│   │       └── package.xml
│   └── build/           # Colcon 编译
│
└── standalone/          # 独立版本
    ├── CMakeLists.txt   # 独立版本
    ├── src/
    └── build/           # CMake 编译
```

## 🔄 实际场景示例

### 场景1: 在 ROS2 项目中开发（您的情况）

```bash
# 步骤1: 写代码
vim example/src/src/custom_sensor_reader.cpp

# 步骤2: 用 Colcon 编译
cd example
colcon build

# 步骤3: 运行
source install/setup.bash
ros2 run unitree_ros2_example custom_sensor_reader
```

**不要混用 cmake：**
```bash
# ❌ 不要这样做
cd example/build/unitree_ros2_example
cmake ../../src
make
# 这会导致路径混乱
```

### 场景2: 提取代码做独立程序

**如果想把传感器读取器做成独立程序：**

```bash
# 创建独立项目
mkdir -p ~/my_standalone_sensor_reader
cd ~/my_standalone_sensor_reader

# 复制代码
cp /home/weeq/unitree_ros2/example/src/src/custom_sensor_reader.cpp ./

# 创建独立的 CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(standalone_sensor_reader)

set(CMAKE_CXX_STANDARD 14)

# 手动查找依赖
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(sensor_reader custom_sensor_reader.cpp)
target_link_libraries(sensor_reader 
    rclcpp::rclcpp
    ${unitree_go_LIBRARIES}
)

install(TARGETS sensor_reader DESTINATION bin)
EOF

# 用 CMake 编译
mkdir build && cd build
cmake ..
make
```

## 💡 最佳实践建议

### 对于您的 unitree_ros2 项目：

#### ✅ 推荐做法
```bash
# 始终使用 Colcon
cd /home/weeq/unitree_ros2/example
colcon build
```

#### ❌ 避免做法
```bash
# 不要混用
cd build/unitree_ros2_example
cmake ../../src  # 会破坏 colcon 的结构
```

### 清理编译文件的正确方法：

```bash
# 完全清理
cd /home/weeq/unitree_ros2/example
rm -rf build/ install/ log/

# 重新编译
colcon build
```

## 🚀 快速命令参考

### Colcon 常用命令

```bash
# 编译所有包
colcon build

# 编译特定包
colcon build --packages-select unitree_ros2_example

# 编译时显示详细输出
colcon build --event-handlers console_direct+

# 清理后重新编译
rm -rf build install log
colcon build

# 只编译修改过的包
colcon build --packages-up-to unitree_ros2_example

# 并行编译（4线程）
colcon build --parallel-workers 4
```

### CMake 常用命令（仅供参考）

```bash
# 独立项目编译
mkdir build && cd build
cmake ..
make

# 指定构建类型
cmake -DCMAKE_BUILD_TYPE=Release ..
make

# 清理
cd .. && rm -rf build/
```

## 📝 总结

### 核心要点：

1. **Colcon 内部就是调用 CMake**
   - Colcon 是更高层的封装
   - CMakeLists.txt 仍然是构建配置文件

2. **编译产物理论上相同**
   - 都生成可执行文件和库
   - 但路径和环境配置不同

3. **不建议混用的原因**
   - 目录结构冲突
   - 环境变量冲突
   - 依赖管理混乱

4. **推荐方案**
   - ROS2 项目 → 一直用 Colcon
   - 独立项目 → 一直用 CMake
   - 需要两者 → 分开维护

### 对于您的项目：

```bash
# 正确的工作流程
cd /home/weeq/unitree_ros2/example
colcon build                              # ✅
source install/setup.bash                 # ✅
ros2 run unitree_ros2_example custom_sensor_reader  # ✅
```

**一句话总结：** 在 ROS2 项目中，坚持使用 Colcon，不要直接调用 CMake。Colcon 会帮你正确地使用 CMake。
