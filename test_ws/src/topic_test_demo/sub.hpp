#ifndef PUB_SUB_DEMO__SUBSCRIBER_NODE_HPP_
#define PUB_SUB_DEMO__SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode();

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg) const;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // PUB_SUB_DEMO__SUBSCRIBER_NODE_HPP_
