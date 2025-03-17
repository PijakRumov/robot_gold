#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>

class LineFollow : public rclcpp::Node{
public:
    LineFollow();
    ~LineFollow() = default;

    void motor_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void motor_driver(float center_distance);


private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr center_distance_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned char left_motor;
    unsigned char right_motor;
};