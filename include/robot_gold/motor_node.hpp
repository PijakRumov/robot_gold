#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "std_msgs/msg/u_int8.hpp"

class MotorNode : public rclcpp::Node
{
public:
    MotorNode();

private:
    void publish_motor_speed();
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
