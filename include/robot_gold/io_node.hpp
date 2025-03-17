#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        IoNode();
        ~IoNode() override = default;

        int get_button_pressed() const;
        void update_leds(int button_pressed);
        void timer_callback();

    private:
        int button_pressed_ = -1;
        std::vector<uint8_t> leds_array{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    };
}