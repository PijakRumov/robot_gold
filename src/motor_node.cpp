#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "motor_node.hpp"
#include "encoder_node.hpp"
#include <cmath>

struct RobotSpeed{
    double v; // linear velocity
    double w; // angular velocity
};

struct WheelSpeed{
    double l; // left wheel velocity
    double r; // right wheel velocity
};

struct Encoders{
    int l; // left encoder count
    int r; // right encoder count
};

struct Coordinates{
    double x; 
    double y;
};

class Kinematics{
public:
    // Forward kinematics: from wheel speeds to robot speed
    RobotSpeed forward(WheelSpeed wheels) {
        RobotSpeed speed;
        speed.v = (R / 2.0) * (wheels.r + wheels.l);  // Linear velocity: (R/2) * (w_r + w_l)
        speed.w = (R / L) * (wheels.r - wheels.l);  // Angular velocity: (R/L) * (w_r - w_l)
        return speed;
    }

    // Inverse kinematics: from robot speed to wheel speeds
    WheelSpeed inverse(RobotSpeed robot) {
        WheelSpeed wheels;
        wheels.l = (2 * robot.v - robot.w * L) / (2 * R);  // (2v - wL) / (2R)
        wheels.r = (2 * robot.v + robot.w * L) / (2 * R);  // (2v + wL) / (2R)
        return wheels;
    }

    // Forward kinematics: from encoder readings to position (x, y)
    Coordinates forward(Encoders encoders) {
        Coordinates coords;
        double d_left = (encoders.l / 576.0) * 2 * M_PI * R;  // Left wheel distance traveled
        double d_right = (encoders.r / 576.0) * 2 * M_PI * R;  // Right wheel distance traveled
        coords.x = ((d_left + d_right) / 2) * cos(phi);  // Calculate X based on both wheel displacements
        coords.y = ((d_left + d_right) / 2) * sin(phi);  // Calculate Y based on both wheel displacements
        return coords;
    }

    // Inverse kinematics: from coordinates to encoder values (approximated)
    Encoders inverse(Coordinates coords) {
        Encoders encoders;
        // Approximate encoder readings based on position change
        double d = sqrt(coords.x * coords.x + coords.y * coords.y);
        encoders.l = (d / (2 * M_PI * R)) * 576;  // Inversely calculate left encoder count
        encoders.r = (d / (2 * M_PI * R)) * 576;  // Inversely calculate right encoder count
        return encoders;
    }

private:
    static constexpr double R = 0.03291;  // Wheel radius (32 mm)
    static constexpr double L = 0.126;  // Wheel base (100 mm)
    static constexpr double phi = 10.0;  // Robot orientation, assuming initial orientation is 0
};

MotorNode::MotorNode()
: Node("motor_node")
{
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MotorNode::publish_motor_speed, this)
    );
}

EncoderNode::EncoderNode()
: Node("encoder_node")
{
    subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>("/bpc_prp_robot/encoders", 10,
        std::bind(&EncoderNode::subscribe_encoder_state, this, std::placeholders::_1)
    );
}

void EncoderNode::subscribe_encoder_state(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) //prave kole -> [1] leve kolo -> [0]
{
    Kinematics kinematics;
    Encoders encoders;
    encoders.l = msg->data[0];
    encoders.r = msg->data[1];

    Coordinates coords = kinematics.forward(encoders);

    RCLCPP_INFO(this->get_logger(), "Received encoder left value: %d", encoders.l);
    RCLCPP_INFO(this->get_logger(), "Received encoder right value: %d", encoders.r);
    RCLCPP_INFO(this->get_logger(), "Calculated X coordinate: %f", coords.x);
    RCLCPP_INFO(this->get_logger(), "Calculated Y coordinate: %f", coords.y);
}

void MotorNode::publish_motor_speed()
{
    auto message = std_msgs::msg::UInt8MultiArray();
    message.data = {255, 200};  // Example motor speeds
    
    std::string data_str;
    for (const auto & value : message.data) {
        data_str += std::to_string(value) + " ";
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", data_str.c_str());
    publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto motor_node = std::make_shared<MotorNode>();
    auto encoder_node = std::make_shared<EncoderNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motor_node);
    executor.add_node(encoder_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}