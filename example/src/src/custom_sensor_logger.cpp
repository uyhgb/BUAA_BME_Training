/**
 * 自定义传感器数据记录器
 * 本示例展示如何将传感器数据保存到文件，或转发到自定义话题
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>

class SensorDataLogger : public rclcpp::Node {
 public:
  SensorDataLogger() : Node("sensor_data_logger"), 
                       sample_count_(0),
                       log_enabled_(true) {
    
    // 创建日志文件
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "sensor_data_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
    log_filename_ = ss.str();
    
    log_file_.open(log_filename_);
    if (log_file_.is_open()) {
      // 写入CSV文件头
      log_file_ << "timestamp,roll,pitch,yaw,ax,ay,az,wx,wy,wz,";
      log_file_ << "motor0_pos,motor0_vel,motor0_torque,";
      log_file_ << "battery_voltage,battery_current\n";
      RCLCPP_INFO(this->get_logger(), "日志文件已创建: %s", log_filename_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "无法创建日志文件!");
      log_enabled_ = false;
    }
    
    // 订阅原始传感器话题
    sensor_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "lf/lowstate", 10,
        std::bind(&SensorDataLogger::sensor_callback, this, std::placeholders::_1));
    
    // 创建自定义发布者 - 可以发布处理后的数据
    custom_pub_ = this->create_publisher<std_msgs::msg::String>("custom_sensor_data", 10);
    
    RCLCPP_INFO(this->get_logger(), "传感器数据记录器已启动");
  }
  
  ~SensorDataLogger() {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(this->get_logger(), "日志文件已关闭，共记录 %ld 条数据", sample_count_);
    }
  }

 private:
  void sensor_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    sample_count_++;
    
    // 每隔一定时间记录一次 (例如每10次记录一次)
    if (sample_count_ % 10 != 0) {
      return;
    }
    
    // 获取当前时间戳
    auto timestamp = this->now().seconds();
    
    // 提取需要的传感器数据
    const auto& imu = msg->imu_state;
    const auto& motor0 = msg->motor_state[0];
    
    // 保存到文件
    if (log_enabled_ && log_file_.is_open()) {
      log_file_ << std::fixed << std::setprecision(6)
                << timestamp << ","
                << imu.rpy[0] << "," << imu.rpy[1] << "," << imu.rpy[2] << ","
                << imu.accelerometer[0] << "," << imu.accelerometer[1] << "," << imu.accelerometer[2] << ","
                << imu.gyroscope[0] << "," << imu.gyroscope[1] << "," << imu.gyroscope[2] << ","
                << motor0.q << "," << motor0.dq << "," << motor0.tau_est << ","
                << msg->power_v << "," << msg->power_a << "\n";
    }
    
    // 发布自定义消息到新话题
    auto custom_msg = std_msgs::msg::String();
    std::stringstream ss;
    ss << "IMU: [" << imu.rpy[0] << ", " << imu.rpy[1] << ", " << imu.rpy[2] << "] "
       << "Battery: " << msg->power_v << "V";
    custom_msg.data = ss.str();
    custom_pub_->publish(custom_msg);
    
    // 打印摘要信息
    if (sample_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "已记录 %ld 条数据", sample_count_);
    }
  }

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sensor_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr custom_pub_;
  
  std::ofstream log_file_;
  std::string log_filename_;
  size_t sample_count_;
  bool log_enabled_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataLogger>());
  rclcpp::shutdown();
  return 0;
}
