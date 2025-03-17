#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "help.hpp"
#include "io_node.hpp"
#include <chrono>
#include <functional>


namespace nodes {
    IoNode::IoNode() : Node("io_node") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            Topic::buttons, 10,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            Topic::set_rgb_leds, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&IoNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Comm. established");
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", button_pressed_);
        update_leds(button_pressed_);
    }

    void IoNode::timer_callback() {
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = leds_array;
        led_publisher_->publish(msg);
    }

    void IoNode::update_leds(int button_pressed) {
        leds_array.clear();
        switch (button_pressed) {
            case 0:
                leds_array = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 0, 255};
            break;
            case 1:
                for(int i = 0; i < 12; i++) {
                    std::chrono::milliseconds(500);
                    if(i<4)
                        IoNode::leds_array = {255,0,0,0,255,0,0,0,255,255,0,255};
                    if(i<8)
                        IoNode::leds_array = {0,255,0,255,0,0,0,255,0,255,0,255};
                }
            break;
            default:
                leds_array = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 0, 255};
            break;
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nodes::IoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/* <rclcpp/rclcpp.hpp>
#include <iostream>
#include "robot_gold/help.hpp"
#include "robot_gold/io_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IoNode>());
    rclcpp::shutdown();

    auto io_node = std::make_shared<nodes::IoNode>();

    executor->add_nodes(io_node);

    while (rclcpp::ok()){
    switch(io_node->get_button_pressed()){
        case 0:

        case 1:

        case 2:

        default:
    }
    }


    return 0;
}
*/
