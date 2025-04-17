#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include <utility>
#include <atomic>

class EncoderNode : public rclcpp::Node
{
public:
    EncoderNode();
    // Destructor (default)
    ~EncoderNode()=default;

    EncoderNode(const EncoderNode&) = delete;             // Delete copy constructor
    EncoderNode& operator=(const EncoderNode&) = delete; 
    int get_left_value();
    int get_right_value();
    void resetl();
    void resetr();

private:
    void subscribe_encoder_state(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    //void publish_motor_speed();
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
    //rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int* estimation(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    int encoderValue0_= 0;
    int encoderValue1_= 0;
    //std::atomic<int> encoderValue0_{0};
    //std::atomic<int> encoderValue1_{0};
};
