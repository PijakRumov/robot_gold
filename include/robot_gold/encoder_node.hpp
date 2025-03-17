#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "std_msgs/msg/u_int8.hpp"

class EncoderNode : public rclcpp::Node
{
public:
    EncoderNode();

private:
    void subscribe_encoder_state(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    //void publish_motor_speed();
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
    //rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int* estimation(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
};
