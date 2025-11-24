/**
 * 模拟机器人低层状态发布器
 * 用于在没有实际机器人连接时测试传感器读取程序
 * 这个程序会模拟发布机器人的低层状态数据
 **/
#include <array>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"

using namespace std::chrono_literals;

class SimulatedRobotPublisher : public rclcpp::Node {
 public:
  SimulatedRobotPublisher() : Node("simulated_robot_publisher"), count_(0) {
    // 创建发布者 - 发布低频低层状态
    publisher_ = this->create_publisher<unitree_go::msg::LowState>("lf/lowstate", 10);
    
    // 创建定时器，每100ms发布一次（10Hz）
    timer_ = this->create_wall_timer(
        100ms, std::bind(&SimulatedRobotPublisher::publish_state, this));
    
    RCLCPP_INFO(this->get_logger(), "🤖 模拟机器人状态发布器已启动");
    RCLCPP_INFO(this->get_logger(), "📡 发布话题: /lf/lowstate (10Hz)");
    RCLCPP_INFO(this->get_logger(), "💡 可以运行 custom_sensor_reader 来接收数据");
  }

 private:
  void publish_state() {
    auto msg = unitree_go::msg::LowState();
    
    // 模拟时间（秒）
    double t = count_ * 0.1;
    
    // ========== 模拟 IMU 数据 ==========
    msg.imu_state.rpy[0] = 0.05 * std::sin(t * 0.5);  // roll
    msg.imu_state.rpy[1] = 0.03 * std::cos(t * 0.5);  // pitch
    msg.imu_state.rpy[2] = t * 0.1;                    // yaw (持续旋转)
    
    // 四元数（简化，实际应该从欧拉角正确转换）
    msg.imu_state.quaternion[0] = 1.0;
    msg.imu_state.quaternion[1] = 0.0;
    msg.imu_state.quaternion[2] = 0.0;
    msg.imu_state.quaternion[3] = 0.0;
    
    // 陀螺仪（角速度）
    msg.imu_state.gyroscope[0] = 0.025 * std::cos(t * 0.5);
    msg.imu_state.gyroscope[1] = -0.015 * std::sin(t * 0.5);
    msg.imu_state.gyroscope[2] = 0.1;
    
    // 加速度计
    msg.imu_state.accelerometer[0] = 0.1 * std::sin(t);
    msg.imu_state.accelerometer[1] = 0.1 * std::cos(t);
    msg.imu_state.accelerometer[2] = 9.81;  // 重力加速度
    
    // ========== 模拟电机数据（12个电机）==========
    for (int i = 0; i < 12; i++) {
      msg.motor_state[i].mode = 0x01;  // FOC模式
      
      // 模拟正弦波运动
      msg.motor_state[i].q = 0.5 * std::sin(t + i * 0.5);  // 位置
      msg.motor_state[i].dq = 0.5 * std::cos(t + i * 0.5);  // 速度
      msg.motor_state[i].ddq = -0.5 * std::sin(t + i * 0.5); // 加速度
      
      // 模拟扭矩（基于位置的简单反馈）
      msg.motor_state[i].tau_est = -10.0 * msg.motor_state[i].q;
      
      // 温度（30-50度之间变化）
      msg.motor_state[i].temperature = static_cast<int8_t>(40 + 5 * std::sin(t * 0.1 + i));
      
      msg.motor_state[i].lost = 0;
    }
    
    // 其他电机设置为0
    for (int i = 12; i < 20; i++) {
      msg.motor_state[i].mode = 0x00;
      msg.motor_state[i].q = 0.0;
      msg.motor_state[i].dq = 0.0;
      msg.motor_state[i].ddq = 0.0;
      msg.motor_state[i].tau_est = 0.0;
      msg.motor_state[i].temperature = 25;
    }
    
    // ========== 模拟足端力 ==========
    for (int i = 0; i < 4; i++) {
      // 模拟站立时的足端力（200-300之间变化）
      msg.foot_force[i] = static_cast<int16_t>(250 + 50 * std::sin(t + i * 1.57));
      msg.foot_force_est[i] = msg.foot_force[i] + static_cast<int16_t>(10 * std::sin(t * 5));
    }
    
    // ========== 模拟电池数据 ==========
    msg.power_v = 24.5 - 0.5 * (count_ % 100) / 100.0;  // 电压 24.5V -> 24V
    msg.power_a = 2.0 + 0.5 * std::sin(t);               // 电流 1.5A -> 2.5A
    
    // ========== 其他字段 ==========
    msg.level_flag = 0xFF;
    msg.tick = count_;
    
    // 发布消息
    publisher_->publish(msg);
    
    // 每10次打印一次摘要
    if (count_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "📊 已发布 %d 条消息 | IMU: [%.3f, %.3f, %.3f] | 电池: %.2fV, %.2fA",
                  count_, 
                  msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2],
                  msg.power_v, msg.power_a);
    }
    
    count_++;
  }

  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "");
  RCLCPP_INFO(rclcpp::get_logger("main"), "========================================");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   模拟宇树机器人状态发布器");
  RCLCPP_INFO(rclcpp::get_logger("main"), "========================================");
  RCLCPP_INFO(rclcpp::get_logger("main"), "");
  RCLCPP_INFO(rclcpp::get_logger("main"), "💡 使用说明：");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   1. 此程序模拟机器人发布低层状态数据");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   2. 在另一个终端运行:");
  RCLCPP_INFO(rclcpp::get_logger("main"), "      ros2 run unitree_ros2_example custom_sensor_reader");
  RCLCPP_INFO(rclcpp::get_logger("main"), "   3. 或使用 ros2 topic echo /lf/lowstate");
  RCLCPP_INFO(rclcpp::get_logger("main"), "");
  
  rclcpp::spin(std::make_shared<SimulatedRobotPublisher>());
  rclcpp::shutdown();
  return 0;
}
