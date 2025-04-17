#include <cmath>
#include <vector>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "pid.hpp"

namespace algorithms {

    struct LidarFiltrResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFiltr {
    public:
        LidarFiltr() = default;

        LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
            constexpr float angle_range = M_PI / 6;

            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            auto angle_step = (angle_end - angle_start) / points.size();
            RCLCPP_DEBUG(rclcpp::get_logger("LidarFiltr"), "Starting filter application...");
            RCLCPP_DEBUG(rclcpp::get_logger("LidarFiltr"), "Angle range: [%f, %f], Step: %f", angle_start, angle_end, angle_step);

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                if (std::isinf(points[i])) {
                    continue;
                }

                if (angle >= -angle_range && angle <= angle_range) {
                    back.push_back(points[i]);
                }if (angle > M_PI / 2 - angle_range && angle <= M_PI / 2 + angle_range) {
                    right.push_back(points[i]);
                }if (angle > -M_PI / 2 - angle_range && angle <= -M_PI / 2 + angle_range) {
                    left.push_back(points[i]);
                }if (angle < -M_PI + angle_range || angle > M_PI - angle_range) {
                    front.push_back(points[i]);
                }
            }

            RCLCPP_DEBUG(rclcpp::get_logger("LidarFiltr"), "Filter results: Front: %zu, Back: %zu, Left: %zu, Right: %zu",
                         front.size(), back.size(), left.size(), right.size());

            return LidarFiltrResults{
                .front = front.empty() ? 0.0f : std::accumulate(front.begin(), front.end(), 0.0f) / front.size(),
                .back = back.empty() ? 0.0f : std::accumulate(back.begin(), back.end(), 0.0f) / back.size(),
                .left = left.empty() ? 0.0f : std::accumulate(left.begin(), left.end(), 0.0f) / left.size(),
                .right = right.empty() ? 0.0f : std::accumulate(right.begin(), right.end(), 0.0f) / right.size(),
            };
        }
    };
}

class LidarNode : public rclcpp::Node {
public:
    LidarNode() : Node("lidar_node") {
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&LidarNode::lidar_callback, this, std::placeholders::_1));

        filtered_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filtered_lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "Initializing LidarNode...");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> lidar_points = msg->ranges;

        algorithms::LidarFiltr filter;
        auto results = filter.apply_filter(lidar_points, msg->angle_min, msg->angle_max);

        auto filtered_msg = std_msgs::msg::Float32MultiArray();
        filtered_msg.data = {
            (results.front),
            (results.back),
            (results.left),
            (results.right)
        };

        filtered_publisher_->publish(filtered_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published filtered distances: Front: %f, Back: %f, Left: %f, Right: %f",
                    results.front, results.back,
                    results.left, results.right);

        int corridor_turn = 0;     // definice ze zatacky nejsou videt 0 je bez zatacky 1 je zatacka vpravo 2 vlevo 3 obe
        

        algorithms::Pid pid(0.5, 0.3, 0.0);
        float center_distance = 0;
        if(results.right>0.3){       // thresholdy at ofiltruju metry
            results.right = 0.2;
            corridor_turn = corridor_turn + 1;
        }
        if(results.left>0.3){
            results.left = 0.2;
            corridor_turn = corridor_turn + 2;
        }

        if(results.left > 0 && results.right > 0){      // kdyz jsem cca ve stredu delej tohle
            center_distance = ((results.right-results.left)/2)*100; // prevod na metry{}
            RCLCPP_INFO(this->get_logger(), "Center distance: %f", center_distance);
        }
        else if(results.right > 0){                     // jsem moc blizko prave steny
            center_distance = -15;
        }
        else if(results.left > 0){                                           // jsem moc blizko leve steny
            center_distance = 15;
        }
        else
        {
            center_distance = 0;
        }
        
        
        unsigned char left_motor;
        unsigned char right_motor;
        auto message = std_msgs::msg::UInt8MultiArray();

        float x = pid.step(center_distance, 0.2);
        RCLCPP_INFO(this->get_logger(), "PID output: %f", x); //p 0.4 i 0.2 d 0.0 dt 0.2 funkcni by me
        if(x>0){
            left_motor = 142;
            right_motor = 142-x;
            message.data = {left_motor, right_motor};
        }
        else {
            left_motor = 142+x;
            right_motor = 142;
            message.data = {left_motor, right_motor};
        }
                                
        motor_publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filtered_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;
};