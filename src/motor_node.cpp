#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "motor_node.hpp"
#include "encoder_node.hpp"
#include <cmath>
#define R 0.033
#define L 0.14

/*
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
*/
/*
int EncoderNode::get_left_value()
{
    return encoderValue0_.load();
}

int EncoderNode::get_right_value()
{
    return encoderValue1_.load();
}

void EncoderNode::resetl()
{
    encoderValue0_.store(0);
}

void EncoderNode::resetr()
{
    encoderValue1_.store(0);
}
*/


double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

namespace algorithms {

    KinematicsAlgorithms::KinematicsAlgorithms()= default;
    KinematicsAlgorithms::~KinematicsAlgorithms()= default;

    Coordinates KinematicsAlgorithms::Forward_odometry (const Encoders in)
    {
        auto delta_dl = R * ((2*M_PI*in.l)/576);
        auto delta_dr = R * ((2*M_PI*in.r)/576);

        auto delta_d = (delta_dl + delta_dr)/2;
        auto delta_fi = (delta_dr - delta_dl)/L;

        Coordinates out{};
        out.x = delta_d * std::cos(delta_fi/2);
        out.y = delta_d * std::sin(delta_fi/2);


        return out;
    }

    Encoders KinematicsAlgorithms::Inverse_odometry (const Coordinates in)
    {
        Encoders out{};


        return out;
    }

    WheelSpeed KinematicsAlgorithms::Inverse_kinematics (const RobotSpeed in)
    {
        WheelSpeed out{};
        out.r = (2*in.v + in.w*L)/(2*R);
        out.l = (2*in.v - in.w*L)/(2*R);
        return out;
    }

    RobotSpeed KinematicsAlgorithms::Forward_kinematics (const WheelSpeed in)
    {
        RobotSpeed out{};
        out.w = (R * in.r -R * in.l)/L;
        out.v = (R * in.l/2 + R * in.r/2);
        return out;
    }

    Pose KinematicsAlgorithms::update_pose(const Pose& current_pose, const Encoders in) {

        Pose out{};
        auto delta_dl = R * ((2*M_PI*in.l)/576);
        auto delta_dr = R * ((2*M_PI*in.r)/576);

        auto  delta_d = (delta_dl + delta_dr)/2;
        auto delta_fi = (delta_dr - delta_dl)/L;

        out.x = current_pose.x + delta_d * std::cos(toRadians(current_pose.theta + delta_fi/2));
        out.y = current_pose.y + delta_d * std::sin(toRadians(current_pose.theta + delta_fi/2));
        out.theta = current_pose.theta + delta_fi;
        return out;
    }
}

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
    //Kinematics kinematics;
    //Encoders encoders;
    //encoders.l = msg->data[0];
    //encoders.r = msg->data[1];

    //Coordinates coords = kinematics.forward(encoders);
    //algorithms::KinematicsAlgorithms kinematics;
    /*
    Coordinates coords = kinematics.Forward_odometry(encoders);
    int x = coords.x;
    int y = coords.y;
    */
   //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
   Coordinates coordinates{0 ,0};
   WheelSpeed wheel_speed{};
   RobotSpeed robot_speed{10, 0.5};
   Encoders encoders{0, 0};
   Encoders tmp_encoders{0, 0};
   Pose pose{};

   encoderValue0_ = msg->data[0];
   encoderValue1_ = - msg->data[1];


   wheel_speed = algorithms::KinematicsAlgorithms::Inverse_kinematics(robot_speed);
   encoders.l = encoderValue0_ - tmp_encoders.l;
   encoders.r = encoderValue1_ - tmp_encoders.r;
   //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
   tmp_encoders.l = encoderValue0_;
   tmp_encoders.r = encoderValue1_;

   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

   // Vypocet nove pozy
   pose = algorithms::KinematicsAlgorithms::update_pose(pose, encoders);
   std::cout << pose.x << " m, " << pose.y << " m, " << pose.theta << " rad" << std::endl;

    RCLCPP_INFO(this->get_logger(), "Received encoder left value: %d", encoders.l);
    RCLCPP_INFO(this->get_logger(), "Received encoder right value: %d", encoders.r);
    RCLCPP_INFO(this->get_logger(), "X: %f", pose.x);
    RCLCPP_INFO(this->get_logger(), "Y: %f", pose.y);
    //RCLCPP_INFO(this->get_logger(), "Calculated X coordinate: %f", coords.x);
    //RCLCPP_INFO(this->get_logger(), "Calculated Y coordinate: %f", coords.y);
}

void MotorNode::publish_motor_speed()
{
    auto message = std_msgs::msg::UInt8MultiArray();
    message.data = {135, 200};  // Example motor speeds
    
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

    //rclcpp::spin(encoder_node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motor_node);
    executor.add_node(encoder_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}