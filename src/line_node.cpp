#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "line_node.hpp"
#include <iostream>

LineNode::LineNode() : Node("line_node") {
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1)
    );
    center_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/bpc_prp_robot/central_distance", 10);

    RCLCPP_INFO(this->get_logger(), "Line node started");
}

void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    float left_value = msg->data[0];
    float right_value = msg->data[1];

    RCLCPP_INFO(this->get_logger(), "Continuous line pose: %f %f", left_value, right_value);

    LineNode::estimate_continuous_line_pose(left_value, right_value);
}

float LineNode::estimate_continuous_line_pose(float left_value, float right_value){
    float y_left = (left_value - 25)/(640 - 25);
    float y_right = -(right_value - 160)/(1000 - 160);

    float y = y_left + y_right;
    
    RCLCPP_INFO(this->get_logger(), "Result of y: %f", y);

    float center_distance = -y*15;

    RCLCPP_INFO(this->get_logger(), "Central distance in mm: %f", center_distance);

    std_msgs::msg::Float32 msg;
    msg.data = center_distance;
    center_distance_publisher_->publish(msg);

    return center_distance;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto line_node = std::make_shared<LineNode>();

    rclcpp::spin(line_node);

    rclcpp::shutdown();

    return 0;
}

