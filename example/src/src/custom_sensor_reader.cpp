/**
 * 自定义传感器读取示例
 * 本示例展示如何订阅宇树机器人的传感器话题，并提取您需要的传感器信息
 * 您可以根据自己的需求修改此代码
 **/
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_state.hpp"


// ============ 配置区域 - 根据需要修改 ============
constexpr bool READ_IMU = true;         // 是否读取IMU数据
constexpr bool READ_MOTOR = true;       // 是否读取电机数据
constexpr bool READ_FOOT_FORCE = false; // 是否读取足端力传感器
constexpr bool READ_BATTERY = true;     // 是否读取电池信息
constexpr bool USE_HIGH_FREQ = false;   // 是否使用高频率(500Hz)，false为低频

// 可以选择只读取特定的电机 (0-11)
const std::vector<int> SELECTED_MOTORS = {0, 1, 2, 3}; // 只读取前4个电机

class CustomSensorReader : public rclcpp::Node {
 public:
  CustomSensorReader() : Node("custom_sensor_reader") {
    // 设置订阅话题
    std::string topic_name;
    if (USE_HIGH_FREQ) {
      topic_name = "lowstate";  // 高频话题 500Hz
    } else {
      topic_name = "lf/lowstate";  // 低频话题
    }
    
    RCLCPP_INFO(this->get_logger(), "启动自定义传感器读取节点...");
    RCLCPP_INFO(this->get_logger(), "订阅话题: %s", topic_name.c_str());
    
    // 创建订阅者
    subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
        topic_name, 10,
        std::bind(&CustomSensorReader::sensor_callback, this, std::placeholders::_1));
  }

 private:
  void sensor_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    // ========== IMU传感器数据读取 ==========
    if (READ_IMU) {
      process_imu_data(msg->imu_state);
    }
    
    // ========== 电机传感器数据读取 ==========
    if (READ_MOTOR) {
      process_motor_data(msg->motor_state);
    }
    
    // ========== 足端力传感器数据读取 ==========
    if (READ_FOOT_FORCE) {
      process_foot_force_data(msg->foot_force, msg->foot_force_est);
    }
    
    // ========== 电池传感器数据读取 ==========
    if (READ_BATTERY) {
      process_battery_data(msg->power_v, msg->power_a);
    }
    
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
  }
  
  // ========== IMU数据处理函数 ==========
  void process_imu_data(const unitree_go::msg::IMUState& imu) {
    // 欧拉角 (roll, pitch, yaw)
    double roll = imu.rpy[0];
    double pitch = imu.rpy[1];
    double yaw = imu.rpy[2];
    
    // 加速度计
    double ax = imu.accelerometer[0];
    double ay = imu.accelerometer[1];
    double az = imu.accelerometer[2];
    
    RCLCPP_INFO(this->get_logger(), 
                "[IMU] Roll: %.3f, Pitch: %.3f, Yaw: %.3f", 
                roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), 
                "[IMU] 加速度 ax: %.3f, ay: %.3f, az: %.3f", 
                ax, ay, az);
    
    // ===== 在这里添加您自己的IMU数据处理逻辑 =====
    // 例如: 保存到文件、发送到其他话题、进行姿态估计等
    // 如果需要四元数和陀螺仪数据，可以访问:
    // imu.quaternion[0-3] - 四元数 [w, x, y, z]
    // imu.gyroscope[0-2] - 陀螺仪 [wx, wy, wz]
  }
  
  // ========== 电机数据处理函数 ==========
  void process_motor_data(const std::array<unitree_go::msg::MotorState, 20>& motor_state) {
    RCLCPP_INFO(this->get_logger(), "[电机数据]");
    
    for (int motor_id : SELECTED_MOTORS) {
      if (motor_id >= 20) continue;  // 防止越界
      
      const auto& motor = motor_state[motor_id];
      
      // 电机状态数据
      double position = motor.q;         // 关节角度 (rad)
      double velocity = motor.dq;        // 角速度 (rad/s)
      double torque = motor.tau_est;     // 估计扭矩 (N·m)
      int temperature = motor.temperature; // 温度 (°C)
      
      RCLCPP_INFO(this->get_logger(), 
                  "  电机[%d] 位置: %.3f rad, 速度: %.3f rad/s, 扭矩: %.3f N·m, 温度: %d°C",
                  motor_id, position, velocity, torque, temperature);
      
      // ===== 在这里添加您自己的电机数据处理逻辑 =====
      // 例如: 监控电机温度、记录运动轨迹等
      // 如果需要加速度: motor.ddq (rad/s²)
    }
  }
  
  // ========== 足端力传感器数据处理函数 ==========
  void process_foot_force_data(const std::array<int16_t, 4>& foot_force, 
                                const std::array<int16_t, 4>& foot_force_est) {
    RCLCPP_INFO(this->get_logger(), 
                "[足端力] 实测: [%d, %d, %d, %d]", 
                foot_force[0], foot_force[1], foot_force[2], foot_force[3]);
    RCLCPP_INFO(this->get_logger(), 
                "[足端力] 估计: [%d, %d, %d, %d]", 
                foot_force_est[0], foot_force_est[1], 
                foot_force_est[2], foot_force_est[3]);
    
    // ===== 在这里添加您自己的足端力数据处理逻辑 =====
    // 例如: 检测接触状态、步态分析等
  }
  
  // ========== 电池数据处理函数 ==========
  void process_battery_data(float voltage, float current) {
    RCLCPP_INFO(this->get_logger(), 
                "[电池] 电压: %.2f V, 电流: %.2f A, 功率: %.2f W", 
                voltage, current, voltage * current);
    
    // ===== 在这里添加您自己的电池数据处理逻辑 =====
    // 例如: 电量监控、低电量报警等
    if (voltage < 20.0) {
      RCLCPP_WARN(this->get_logger(), "警告: 电池电压过低!");
    }
  }

  // 订阅者
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<CustomSensorReader>();
  
  RCLCPP_INFO(node->get_logger(), "自定义传感器读取节点已启动");
  RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 停止");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
