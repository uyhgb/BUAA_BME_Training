/**
 * 自定义机器人控制器
 * 功能: 订阅自定义的LowState，运行控制算法，输出电机命令
 * 这样你可以用自己的传感器 + 宇树电机搭建机器人
 */

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/motor_cmd.hpp>

class CustomRobotController : public rclcpp::Node
{
public:
    CustomRobotController() : Node("custom_robot_controller")
    {
        RCLCPP_INFO(this->get_logger(), "启动自定义机器人控制器...");
        
        // 声明参数
        this->declare_parameter<int>("num_motors", 12);
        this->declare_parameter<double>("control_frequency", 50.0);
        this->declare_parameter<bool>("enable_control", false);
        
        num_motors_ = this->get_parameter("num_motors").as_int();
        control_freq_ = this->get_parameter("control_frequency").as_double();
        enable_control_ = this->get_parameter("enable_control").as_bool();
        
        // 订阅自定义的LowState
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/custom/lowstate", 10,
            std::bind(&CustomRobotController::lowstate_callback, this, std::placeholders::_1));
        
        // 发布电机命令
        lowcmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>(
            "/lowcmd", 10);
        
        // 控制定时器
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_));
        control_timer_ = this->create_wall_timer(
            period,
            std::bind(&CustomRobotController::control_loop, this));
        
        // 初始化电机命令
        init_motor_cmd();
        
        RCLCPP_INFO(this->get_logger(), "控制器已启动");
        RCLCPP_INFO(this->get_logger(), "电机数量: %d, 控制频率: %.1f Hz", 
                   num_motors_, control_freq_);
        RCLCPP_WARN(this->get_logger(), "控制使能: %s (使用参数enable_control启用)",
                   enable_control_ ? "是" : "否");
    }

private:
    void init_motor_cmd()
    {
        // 初始化LowCmd消息
        lowcmd_msg_.head[0] = 0xFE;
        lowcmd_msg_.head[1] = 0xEF;
        lowcmd_msg_.level_flag = 0xFF;
        
        // 初始化所有电机为待机模式
        for (int i = 0; i < 20; i++) {
            lowcmd_msg_.motor_cmd[i].mode = 0x00;  // 待机模式
            lowcmd_msg_.motor_cmd[i].q = 0.0;
            lowcmd_msg_.motor_cmd[i].dq = 0.0;
            lowcmd_msg_.motor_cmd[i].tau = 0.0;
            lowcmd_msg_.motor_cmd[i].kp = 0.0;
            lowcmd_msg_.motor_cmd[i].kd = 0.0;
        }
    }
    
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        // 保存最新的状态
        latest_state_ = msg;
        state_received_ = true;
        
        // 提取IMU数据
        imu_roll_ = msg->imu_state.rpy[0];
        imu_pitch_ = msg->imu_state.rpy[1];
        imu_yaw_ = msg->imu_state.rpy[2];
        
        // 提取电机状态
        for (int i = 0; i < num_motors_; i++) {
            motor_positions_[i] = msg->motor_state[i].q;
            motor_velocities_[i] = msg->motor_state[i].dq;
            motor_torques_[i] = msg->motor_state[i].tau_est;
        }
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到状态 - IMU: [%.2f, %.2f, %.2f]",
                    imu_roll_, imu_pitch_, imu_yaw_);
    }
    
    void control_loop()
    {
        if (!state_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "等待传感器数据...");
            return;
        }
        
        if (!enable_control_) {
            // 控制未使能，发送待机命令
            send_standby_command();
            return;
        }
        
        // ========== 这里实现你的控制算法 ==========
        
        // 示例1: 简单的姿态平衡控制
        balance_control();
        
        // 示例2: 步态生成
        // gait_generator();
        
        // 示例3: 力控制
        // force_control();
        
        // ========================================
        
        // 发布电机命令
        lowcmd_pub_->publish(lowcmd_msg_);
        
        control_counter_++;
        if (control_counter_ % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "控制中 - Roll: %.3f, Pitch: %.3f",
                       imu_roll_, imu_pitch_);
        }
    }
    
    // ========== 示例控制算法 ==========
    
    void send_standby_command()
    {
        // 发送待机命令 (所有电机阻尼模式)
        for (int i = 0; i < num_motors_; i++) {
            lowcmd_msg_.motor_cmd[i].mode = 0x00;  // 待机
            lowcmd_msg_.motor_cmd[i].q = 0.0;
            lowcmd_msg_.motor_cmd[i].dq = 0.0;
            lowcmd_msg_.motor_cmd[i].tau = 0.0;
            lowcmd_msg_.motor_cmd[i].kp = 0.0;
            lowcmd_msg_.motor_cmd[i].kd = 3.0;  // 小阻尼
        }
        
        lowcmd_pub_->publish(lowcmd_msg_);
    }
    
    void balance_control()
    {
        // 简单的PD平衡控制示例
        // 目标: 保持机身水平
        
        double kp_balance = 50.0;  // 比例增益
        double kd_balance = 5.0;   // 微分增益
        
        // 计算姿态误差
        double roll_error = 0.0 - imu_roll_;   // 目标roll为0
        double pitch_error = 0.0 - imu_pitch_; // 目标pitch为0
        
        // 根据姿态误差调整电机命令
        // 这里是简化示例，实际需要根据你的机器人结构设计
        for (int i = 0; i < num_motors_; i++) {
            lowcmd_msg_.motor_cmd[i].mode = 0x01;  // FOC模式
            
            // 位置控制 + 姿态补偿
            double target_pos = 0.0;  // 基准位置
            
            // 根据腿的位置添加姿态补偿
            if (i < 3) {  // 前左腿
                target_pos += pitch_error * 0.1 - roll_error * 0.1;
            } else if (i < 6) {  // 前右腿
                target_pos += pitch_error * 0.1 + roll_error * 0.1;
            } else if (i < 9) {  // 后左腿
                target_pos -= pitch_error * 0.1 - roll_error * 0.1;
            } else if (i < 12) {  // 后右腿
                target_pos -= pitch_error * 0.1 + roll_error * 0.1;
            }
            
            lowcmd_msg_.motor_cmd[i].q = target_pos;
            lowcmd_msg_.motor_cmd[i].dq = 0.0;
            lowcmd_msg_.motor_cmd[i].kp = kp_balance;
            lowcmd_msg_.motor_cmd[i].kd = kd_balance;
            lowcmd_msg_.motor_cmd[i].tau = 0.0;
        }
    }
    
    // ========== 成员变量 ==========
    // 订阅者和发布者
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 状态数据
    unitree_go::msg::LowState::SharedPtr latest_state_;
    unitree_go::msg::LowCmd lowcmd_msg_;
    
    // IMU数据
    double imu_roll_ = 0.0;
    double imu_pitch_ = 0.0;
    double imu_yaw_ = 0.0;
    
    // 电机数据
    std::array<double, 20> motor_positions_ = {0};
    std::array<double, 20> motor_velocities_ = {0};
    std::array<double, 20> motor_torques_ = {0};
    
    // 参数
    int num_motors_;
    double control_freq_;
    bool enable_control_;
    
    // 状态标志
    bool state_received_ = false;
    int control_counter_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomRobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
