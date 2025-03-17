#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "line_node.hpp"
#include "line_follow.hpp"
#include <iostream>

LineFollow::LineFollow() : Node("line_follow") {
    motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    center_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/bpc_prp_robot/central_distance", 10,
        std::bind(&LineFollow::motor_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Line follow node started");
}

void LineFollow::motor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float center_distance = msg->data;
    motor_driver(center_distance);
    /*
    std::string data_str;
    for (const auto & value : message.data) {
        data_str += std::to_string(value) + " ";
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", data_str.c_str());
    motor_publisher_->publish(message);
    */
}

void LineFollow::motor_driver(float center_distance){
    auto message = std_msgs::msg::UInt8MultiArray();
    if(abs(center_distance)<12){
        message.data = {132, 132};
    }
    else if (center_distance<-12){
        message.data = {129, 132};
    }
    else {
        message.data = {132, 129};
    }
    
    std::string data_str;
    for (const auto & value : message.data) {
        data_str += std::to_string(value) + " ";
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", data_str.c_str());
    motor_publisher_->publish(message);
    
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto line_follow = std::make_shared<LineFollow>();

    rclcpp::spin(line_follow);

    rclcpp::shutdown();

    return 0;
}