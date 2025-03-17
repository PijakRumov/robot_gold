#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include "std_msgs/msg/float32.hpp"

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineNode : public rclcpp::Node {
public:
    
    LineNode();
    
    ~LineNode() = default;

    float get_continuous_line_pose() const;

    DiscreteLinePose get_discrete_line_pose() const;

private:

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_distance_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

    float estimate_continuous_line_pose(float left_value, float right_value);

    DiscreteLinePose estimate_descrete_line_pose(float l_norm, float r_norm);
};