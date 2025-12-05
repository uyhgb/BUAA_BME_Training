/**
 * 传感器桥接节点
 * 功能: 将你自己的传感器数据(标准ROS2消息)转换为宇树LowState格式
 * 这样你就可以使用宇树的电机控制接口，同时用自己的传感器
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/motor_state.hpp>

class SensorBridgeNode : public rclcpp::Node
{
public:
    SensorBridgeNode() : Node("sensor_bridge_node")
    {
        RCLCPP_INFO(this->get_logger(), "启动传感器桥接节点...");
        
        // ========== 订阅你自己的传感器话题 ==========
        // 订阅外接IMU (来自你的ESP32)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&SensorBridgeNode::imu_callback, this, std::placeholders::_1));
        
        // 订阅关节状态 (如果你有编码器或其他位置传感器)
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SensorBridgeNode::joint_callback, this, std::placeholders::_1));
        
        // 可以添加更多传感器订阅...
        // foot_force_sub_ = ...
        // camera_sub_ = ...
        
        // ========== 发布宇树格式的状态 ==========
        // 发布低频状态 (模拟宇树机器人的lowstate话题)
        lowstate_pub_ = this->create_publisher<unitree_go::msg::LowState>(
            "/custom/lowstate", 10);
        
        // 定时发布 (例如50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SensorBridgeNode::publish_lowstate, this));
        
        // 初始化lowstate消息
        init_lowstate();
        
        RCLCPP_INFO(this->get_logger(), "传感器桥接节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /imu/data, /joint_states");
        RCLCPP_INFO(this->get_logger(), "发布话题: /custom/lowstate");
    }

private:
    void init_lowstate()
    {
        // 初始化头部信息
        lowstate_msg_.head[0] = 0xFE;
        lowstate_msg_.head[1] = 0xEF;
        lowstate_msg_.level_flag = 0xFF;
        
        // 初始化电机状态数组 (假设有12个电机，四足机器人)
        for (int i = 0; i < 20; i++) {
            lowstate_msg_.motor_state[i].mode = 0;
            lowstate_msg_.motor_state[i].q = 0.0;
            lowstate_msg_.motor_state[i].dq = 0.0;
            lowstate_msg_.motor_state[i].ddq = 0.0;
            lowstate_msg_.motor_state[i].tau_est = 0.0;
            lowstate_msg_.motor_state[i].temperature = 25;
        }
        
        // 初始化IMU状态
        for (int i = 0; i < 4; i++) {
            lowstate_msg_.imu_state.quaternion[i] = 0.0;
        }
        lowstate_msg_.imu_state.quaternion[0] = 1.0; // w分量
        
        for (int i = 0; i < 3; i++) {
            lowstate_msg_.imu_state.gyroscope[i] = 0.0;
            lowstate_msg_.imu_state.accelerometer[i] = 0.0;
            lowstate_msg_.imu_state.rpy[i] = 0.0;
        }
        
        // 初始化足端力
        for (int i = 0; i < 4; i++) {
            lowstate_msg_.foot_force[i] = 0;
            lowstate_msg_.foot_force_est[i] = 0;
        }
        
        // 初始化电池信息
        lowstate_msg_.power_v = 24.0;
        lowstate_msg_.power_a = 0.0;
    }
    
    // ========== IMU数据回调 ==========
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 将标准ROS2 IMU消息转换为宇树格式
        
        // 四元数
        lowstate_msg_.imu_state.quaternion[0] = msg->orientation.w;
        lowstate_msg_.imu_state.quaternion[1] = msg->orientation.x;
        lowstate_msg_.imu_state.quaternion[2] = msg->orientation.y;
        lowstate_msg_.imu_state.quaternion[3] = msg->orientation.z;
        
        // 陀螺仪 (角速度)
        lowstate_msg_.imu_state.gyroscope[0] = msg->angular_velocity.x;
        lowstate_msg_.imu_state.gyroscope[1] = msg->angular_velocity.y;
        lowstate_msg_.imu_state.gyroscope[2] = msg->angular_velocity.z;
        
        // 加速度计
        lowstate_msg_.imu_state.accelerometer[0] = msg->linear_acceleration.x;
        lowstate_msg_.imu_state.accelerometer[1] = msg->linear_acceleration.y;
        lowstate_msg_.imu_state.accelerometer[2] = msg->linear_acceleration.z;
        
        // 从四元数计算欧拉角 (Roll, Pitch, Yaw)
        quaternion_to_euler(
            msg->orientation.w, msg->orientation.x,
            msg->orientation.y, msg->orientation.z,
            lowstate_msg_.imu_state.rpy[0],  // roll
            lowstate_msg_.imu_state.rpy[1],  // pitch
            lowstate_msg_.imu_state.rpy[2]   // yaw
        );
        
        imu_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到IMU数据 - Roll: %.3f, Pitch: %.3f, Yaw: %.3f",
                    lowstate_msg_.imu_state.rpy[0],
                    lowstate_msg_.imu_state.rpy[1],
                    lowstate_msg_.imu_state.rpy[2]);
    }
    
    // ========== 关节状态回调 ==========
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 将关节状态映射到电机状态
        // 假设你的关节命名为: joint_0, joint_1, ...
        
        for (size_t i = 0; i < msg->name.size() && i < 20; i++) {
            if (i < msg->position.size()) {
                lowstate_msg_.motor_state[i].q = msg->position[i];
            }
            if (i < msg->velocity.size()) {
                lowstate_msg_.motor_state[i].dq = msg->velocity[i];
            }
            if (i < msg->effort.size()) {
                lowstate_msg_.motor_state[i].tau_est = msg->effort[i];
            }
        }
        
        joint_received_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "收到关节状态数据");
    }
    
    // ========== 定时发布LowState ==========
    void publish_lowstate()
    {
        // 更新时间戳
        lowstate_msg_.tick++;
        
        // 发布状态
        lowstate_pub_->publish(lowstate_msg_);
        
        if (!published_once_) {
            RCLCPP_INFO(this->get_logger(), "开始发布自定义LowState数据");
            RCLCPP_INFO(this->get_logger(), "IMU已接收: %s, 关节已接收: %s",
                       imu_received_ ? "是" : "否",
                       joint_received_ ? "是" : "否");
            published_once_ = true;
        }
    }
    
    // ========== 四元数转欧拉角 ==========
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
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    
    // ========== 成员变量 ==========
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    // 发布者
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 状态消息
    unitree_go::msg::LowState lowstate_msg_;
    
    // 状态标志
    bool imu_received_ = false;
    bool joint_received_ = false;
    bool published_once_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
