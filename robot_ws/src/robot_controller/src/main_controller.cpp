/**
 * 机器人主控制器
 * 
 * 功能:
 * 1. 订阅IMU传感器数据
 * 2. 运行控制算法 (待实现)
 * 3. 发布电机控制命令 (待实现)
 * 
 * 作者: Your Name
 * 日期: 2025-12-05
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "  机器人主控制器启动中...");
        RCLCPP_INFO(this->get_logger(), "=================================");
        
        // ========== 参数声明 ==========
        this->declare_parameter<double>("control_frequency", 50.0);
        this->declare_parameter<bool>("enable_control", false);
        this->declare_parameter<int>("num_motors", 12);
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        
        // 获取参数
        control_freq_ = this->get_parameter("control_frequency").as_double();
        enable_control_ = this->get_parameter("enable_control").as_bool();
        num_motors_ = this->get_parameter("num_motors").as_int();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        
        RCLCPP_INFO(this->get_logger(), "配置参数:");
        RCLCPP_INFO(this->get_logger(), "  - 控制频率: %.1f Hz", control_freq_);
        RCLCPP_INFO(this->get_logger(), "  - 电机数量: %d", num_motors_);
        RCLCPP_INFO(this->get_logger(), "  - 控制使能: %s", enable_control_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  - IMU话题: %s", imu_topic.c_str());
        
        // ========== 订阅传感器数据 ==========
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&RobotController::imu_callback, this, std::placeholders::_1));
        
        // ========== 发布控制命令 ==========
        // 电机命令发布器 (格式待定)
        motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_commands", 10);
        
        // 速度命令发布器 (可选，用于高层控制)
        velocity_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // ========== 控制循环定时器 ==========
        auto period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / control_freq_));
        control_timer_ = this->create_wall_timer(
            period,
            std::bind(&RobotController::control_loop, this));
        
        // 初始化状态
        init_state();
        
        RCLCPP_INFO(this->get_logger(), "=================================");
        RCLCPP_INFO(this->get_logger(), "  控制器初始化完成!");
        RCLCPP_INFO(this->get_logger(), "=================================");
        
        if (!enable_control_) {
            RCLCPP_WARN(this->get_logger(), "=================================");
            RCLCPP_WARN(this->get_logger(), "⚠️  控制未使能，运行在监控模式");
            RCLCPP_WARN(this->get_logger(), "   使用参数 -p enable_control:=true 启用控制");
            RCLCPP_WARN(this->get_logger(), "=================================");
        }
    }

private:
    // ========================================
    // 初始化
    // ========================================
    void init_state()
    {
        // 初始化IMU数据
        imu_roll_ = 0.0;
        imu_pitch_ = 0.0;
        imu_yaw_ = 0.0;
        
        imu_angular_vel_x_ = 0.0;
        imu_angular_vel_y_ = 0.0;
        imu_angular_vel_z_ = 0.0;
        
        imu_linear_acc_x_ = 0.0;
        imu_linear_acc_y_ = 0.0;
        imu_linear_acc_z_ = 0.0;
        
        // 初始化电机命令
        motor_commands_.resize(num_motors_, 0.0);
        
        imu_received_ = false;
        loop_count_ = 0;
    }
    
    // ========================================
    // IMU数据回调
    // ========================================
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 保存原始IMU消息
        latest_imu_msg_ = msg;
        
        // 提取线性加速度
        imu_linear_acc_x_ = msg->linear_acceleration.x;
        imu_linear_acc_y_ = msg->linear_acceleration.y;
        imu_linear_acc_z_ = msg->linear_acceleration.z;
        
        // 提取角速度
        imu_angular_vel_x_ = msg->angular_velocity.x;
        imu_angular_vel_y_ = msg->angular_velocity.y;
        imu_angular_vel_z_ = msg->angular_velocity.z;
        
        // 从四元数计算欧拉角
        quaternion_to_euler(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            imu_roll_,
            imu_pitch_,
            imu_yaw_
        );
        
        if (!imu_received_) {
            RCLCPP_INFO(this->get_logger(), "✓ 已接收IMU数据");
            imu_received_ = true;
        }
        
        // 每秒打印一次传感器数据
        if (loop_count_ % static_cast<int>(control_freq_) == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                        "IMU - Roll: %.3f, Pitch: %.3f, Yaw: %.3f",
                        imu_roll_, imu_pitch_, imu_yaw_);
        }
    }
    
    // ========================================
    // 主控制循环
    // ========================================
    void control_loop()
    {
        loop_count_++;
        
        // 检查传感器数据
        if (!imu_received_) {
            if (loop_count_ % 100 == 0) {  // 每2秒提示一次
                RCLCPP_WARN(this->get_logger(), "等待IMU数据...");
            }
            return;
        }
        
        // 根据使能状态执行不同逻辑
        if (enable_control_) {
            // ========== 控制使能 - 运行控制算法 ==========
            run_control_algorithm();
            publish_motor_commands();
        } else {
            // ========== 控制未使能 - 仅监控模式 ==========
            monitor_mode();
        }
        
        // 定期打印状态信息
        if (loop_count_ % static_cast<int>(control_freq_) == 0) {
            print_status();
        }
    }
    
    // ========================================
    // 控制算法 (待实现)
    // ========================================
    void run_control_algorithm()
    {
        // ========================================
        // TODO: 在这里实现你的控制算法
        // ========================================
        
        // 示例: 读取传感器数据
        // double roll = imu_roll_;
        // double pitch = imu_pitch_;
        // double yaw = imu_yaw_;
        
        // 示例: 简单的平衡控制
        // double roll_error = 0.0 - roll;
        // double pitch_error = 0.0 - pitch;
        
        // 示例: 计算电机命令
        for (int i = 0; i < num_motors_; i++) {
            // motor_commands_[i] = ...;  // 你的算法输出
            motor_commands_[i] = 0.0;  // 默认为0
        }
        
        // ========================================
        // 算法框架示例:
        // 1. 状态估计
        // 2. 轨迹规划
        // 3. 步态生成
        // 4. 力/位置控制
        // 5. 安全检查
        // ========================================
    }
    
    // ========================================
    // 发布电机命令 (待实现)
    // ========================================
    void publish_motor_commands()
    {
        // ========================================
        // TODO: 实现电机命令发布逻辑
        // ========================================
        
        // 创建电机命令消息
        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data = motor_commands_;
        
        // 发布命令
        motor_cmd_pub_->publish(motor_msg);
        
        // 示例: 如果使用速度控制接口
        // auto vel_msg = geometry_msgs::msg::Twist();
        // vel_msg.linear.x = ...;
        // vel_msg.angular.z = ...;
        // velocity_cmd_pub_->publish(vel_msg);
    }
    
    // ========================================
    // 监控模式
    // ========================================
    void monitor_mode()
    {
        // 在监控模式下，不发送控制命令
        // 仅记录和显示传感器数据
        
        // 可以在这里添加数据记录逻辑
        // log_data();
    }
    
    // ========================================
    // 打印状态信息
    // ========================================
    void print_status()
    {
        RCLCPP_INFO(this->get_logger(), 
                   "状态 | Roll: %6.2f° | Pitch: %6.2f° | Yaw: %6.2f° | 循环: %d",
                   imu_roll_ * 180.0 / M_PI,
                   imu_pitch_ * 180.0 / M_PI,
                   imu_yaw_ * 180.0 / M_PI,
                   loop_count_);
    }
    
    // ========================================
    // 工具函数: 四元数转欧拉角
    // ========================================
    void quaternion_to_euler(double w, double x, double y, double z,
                            double& roll, double& pitch, double& yaw)
    {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    
    // ========================================
    // 成员变量
    // ========================================
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 传感器数据
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;
    double imu_angular_vel_x_;
    double imu_angular_vel_y_;
    double imu_angular_vel_z_;
    double imu_linear_acc_x_;
    double imu_linear_acc_y_;
    double imu_linear_acc_z_;
    
    // 控制命令
    std::vector<float> motor_commands_;
    
    // 参数
    double control_freq_;
    bool enable_control_;
    int num_motors_;
    
    // 状态标志
    bool imu_received_;
    int loop_count_;
};

// ========================================
// 主函数
// ========================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotController>();
    
    RCLCPP_INFO(node->get_logger(), "开始运行控制循环...");
    RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 停止\n");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "控制器已停止");
    rclcpp::shutdown();
    
    return 0;
}
