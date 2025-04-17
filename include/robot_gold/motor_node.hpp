#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include <utility>

class MotorNode : public rclcpp::Node
{
public:
    MotorNode();

private:
    void publish_motor_speed();
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

struct Pose {
    double x;
    double y;
    double theta;
};

struct RobotSpeed{
    double v; //linear
    double w; //angluar
};

struct WheelSpeed{
    double l; //left
    double r; //right
};

struct Encoders{
    int l; //left
    int r; //right
};

struct Coordinates{
    double x;
    double y;

};

/*
class Kinematics{
    RobotSpeed forward(WheelSpeed);
    WheelSpeed inverse(RobotSpeed);
    Coordinates forward(Encoders);
    Encoders inverse(Coordinates);
}
*/


namespace algorithms {
    class KinematicsAlgorithms{
    public:
        // Constructor
        KinematicsAlgorithms();
        // Destructor (default)
        ~KinematicsAlgorithms();

        static Coordinates Forward_odometry (Encoders in);
        static Encoders Inverse_odometry (Coordinates in);
        static WheelSpeed Inverse_kinematics (RobotSpeed in);
        static RobotSpeed Forward_kinematics (WheelSpeed in);
        WheelSpeed wheelSpeed_fromEncoders (Encoders in, double T);
        static Pose update_pose(const Pose& current_pose, Encoders in);
    private:
        // Variable to store the last received button press value
        // Subscriber for button press messages

    };
}