#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <algorithm>

#include <iostream>
#include <string>
#include <sstream>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

class IMUSerialPublisher : public rclcpp::Node
{
public:
    IMUSerialPublisher() : Node("imu_serial_publisher")
    {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("publish_rate", 20.0);
        
        // 获取参数
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // 创建发布器
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        // 磁力计发布器已禁用 (如需使用,取消下行注释)
        // mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        
        // 打开串口
        if (!openSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully at %d baud", 
                    serial_port_.c_str(), baud_rate_);
        
        // 创建定时器读取串口数据
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / this->get_parameter("publish_rate").as_double()));
        timer_ = this->create_wall_timer(period, std::bind(&IMUSerialPublisher::readAndPublish, this));
    }
    
    ~IMUSerialPublisher()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool openSerialPort()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            return false;
        }
        
        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        // 设置波特率
        speed_t baud;
        switch (baud_rate_) {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:     baud = B115200; break;
        }
        
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;
        
        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);
        
        return true;
    }
    
    std::string readLine()
    {
        static std::string buffer;  // 静态缓冲区,保留未处理的数据
        char temp[256];
        
        while (rclcpp::ok()) {
            // 尝试读取更多数据
            int n = read(serial_fd_, temp, sizeof(temp) - 1);
            if (n > 0) {
                temp[n] = '\0';
                buffer += temp;
                
                // 查找完整的一行 (以 \n 结尾)
                size_t newline_pos = buffer.find('\n');
                if (newline_pos != std::string::npos) {
                    // 提取一行
                    std::string line = buffer.substr(0, newline_pos);
                    buffer = buffer.substr(newline_pos + 1);  // 保留剩余数据
                    
                    // 去除 \r
                    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                    
                    if (!line.empty()) {
                        return line;
                    }
                }
            } else if (n < 0) {
                // 非阻塞模式下 EAGAIN/EWOULDBLOCK 是正常情况,继续等待
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
                return "";
            }
            // n == 0: 超时,继续读取
        }
        
        return "";
    }
    
    void readAndPublish()
    {
        std::string line = readLine();
        if (line.empty()) {
            return;
        }
        
        // 过滤掉单独的单位标注行 (mg, dps 等)
        // 只处理包含完整 JSON 的行(以 '{' 开头, 以 '}' 结尾)
        if (line.empty() || line[0] != '{') {
            return;
        }
        
        // 去除 JSON 后面的空格和尾部的单位标注
        size_t json_end = line.find_last_of('}');
        if (json_end != std::string::npos) {
            line = line.substr(0, json_end + 1);
        } else {
            // 没有找到结束符 '}', 说明 JSON 不完整
            RCLCPP_WARN(this->get_logger(), "Incomplete JSON received (length: %zu), skipping", line.length());
            return;
        }
        
        try {
            // 解析JSON
            auto j = json::parse(line);
            
            // 创建IMU消息
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = frame_id_;
            
            // 填充线性加速度 (保持原始单位: mg)
            // 注意: IMU 输出单位为 mg (毫重力),算法层需要转换为 m/s²
            if (j.contains("acceleration")) {
                imu_msg.linear_acceleration.x = j["acceleration"]["x"].get<double>();
                imu_msg.linear_acceleration.y = j["acceleration"]["y"].get<double>();
                imu_msg.linear_acceleration.z = j["acceleration"]["z"].get<double>();
            }
            
            // 填充角速度 (保持原始单位: dps)
            // 注意: IMU 输出单位为 dps (度/秒),算法层需要转换为 rad/s
            if (j.contains("gyroscope")) {
                imu_msg.angular_velocity.x = j["gyroscope"]["x"].get<double>();
                imu_msg.angular_velocity.y = j["gyroscope"]["y"].get<double>();
                imu_msg.angular_velocity.z = j["gyroscope"]["z"].get<double>();
            }
            
            // 从欧拉角转换为四元数 (Roll, Pitch, Yaw)
            if (j.contains("orientation")) {
                double roll = j["orientation"]["roll"].get<double>() * M_PI / 180.0;  // 转换为弧度
                double pitch = j["orientation"]["pitch"].get<double>() * M_PI / 180.0;
                double yaw = j["orientation"]["yaw"].get<double>() * M_PI / 180.0;
                
                // 欧拉角转四元数
                double cy = cos(yaw * 0.5);
                double sy = sin(yaw * 0.5);
                double cp = cos(pitch * 0.5);
                double sp = sin(pitch * 0.5);
                double cr = cos(roll * 0.5);
                double sr = sin(roll * 0.5);
                
                imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
                imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
                imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
                imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
            }
            
            // 设置协方差 (未知时设为-1,或设置为已知值)
            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = -1;
            imu_msg.linear_acceleration_covariance[0] = -1;
            
            // 发布IMU数据
            imu_pub_->publish(imu_msg);
            
            /* 磁力计数据发布已禁用
            // 如需使用磁力计,取消下面的注释并在ESP32端启用磁力计数据发送
            if (j.contains("magnetic")) {
                auto mag_msg = sensor_msgs::msg::MagneticField();
                mag_msg.header = imu_msg.header;
                
                // 填充磁场数据 (Tesla)
                mag_msg.magnetic_field.x = j["magnetic"]["x"].get<double>() * 1e-6;  // 从uT转换为T
                mag_msg.magnetic_field.y = j["magnetic"]["y"].get<double>() * 1e-6;
                mag_msg.magnetic_field.z = j["magnetic"]["z"].get<double>() * 1e-6;
                
                mag_msg.magnetic_field_covariance[0] = -1;
                
                mag_pub_->publish(mag_msg);
            }
            */
            
            RCLCPP_DEBUG(this->get_logger(), "Published IMU data");
            
        } catch (const json::parse_error& e) {
            RCLCPP_WARN(this->get_logger(), "JSON parse error: %s, line: %s", e.what(), line.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error processing data: %s", e.what());
        }
    }
    
    // 成员变量
    std::string serial_port_;
    int baud_rate_;
    std::string frame_id_;
    int serial_fd_ = -1;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;  // 磁力计发布器已禁用
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUSerialPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
