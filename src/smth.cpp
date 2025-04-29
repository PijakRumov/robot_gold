#include <cmath>
#include <vector>
#include <numeric>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "pid.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

rclcpp::TimerBase::SharedPtr go_straight_timer_;
rclcpp::Time go_straight_start_time_;

namespace algorithms {

struct LidarFiltrResults {
    float front;
    float back;
    float left;
    float right;
};

class LidarFiltr {
public:
    LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
        constexpr float angle_range = M_PI / 6; //old = pi/12; sensing 45° to all 4 directions
        std::vector<float> left, right, front, back;
        auto angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            auto angle = angle_start + i * angle_step;
            if (std::isinf(points[i])) continue;

            if (angle >= -angle_range && angle <= angle_range)  // vector of distances for each of 4 directions
                back.push_back(points[i]);
            else if (angle > M_PI / 2 - angle_range && angle <= M_PI / 2 + angle_range)
                right.push_back(points[i]);
            else if (angle > -M_PI / 2 - angle_range && angle <= -M_PI / 2 + angle_range)
                left.push_back(points[i]);
            else if (angle < -M_PI + angle_range || angle > M_PI - angle_range)
                front.push_back(points[i]);
        }

        return {    // sum and divide by number of values -> average value returned
            .front = front.empty() ? 0.0f : std::accumulate(front.begin(), front.end(), 0.0f) / front.size(),
            .back = back.empty() ? 0.0f : std::accumulate(back.begin(), back.end(), 0.0f) / back.size(),
            .left = left.empty() ? 0.0f : std::accumulate(left.begin(), left.end(), 0.0f) / left.size(),
            .right = right.empty() ? 0.0f : std::accumulate(right.begin(), right.end(), 0.0f) / right.size()
        };
    }
};

class PlanarImuIntegrator {
public:
    PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

    void update(float gyro_z, double dt) {  // velocity of turning
        theta_ += (gyro_z - gyro_offset_) * dt;
    }

    void setCalibration(std::vector<float> gyro) {
        gyro_offset_ = gyro.empty() ? 0.0f : std::accumulate(gyro.begin(), gyro.end(), 0.0f) / gyro.size();
    }

    float getYaw() const { return theta_; }
    void reset() { theta_ = 0.0f; gyro_offset_ = 0.0f; }

private:
    float theta_, gyro_offset_;
};

} // namespace algorithms

class CorridorRobot : public rclcpp::Node {
public:
    enum State { CALIBRATION, CORRIDOR_FOLLOWING, TURNING, STRAIGHT_DELAY, GOING_STRAIGHT_AFTER_TURN, STOP };

    CorridorRobot() : Node("corridor_robot"), state_(CALIBRATION), yaw_start_(0.0) {
        motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/bpc_prp_robot/lidar", 10, std::bind(&CorridorRobot::lidar_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/bpc_prp_robot/imu", 10, std::bind(&CorridorRobot::imu_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::seconds(8), std::bind(&CorridorRobot::switch_to_following, this));
    }

private:
    void switch_to_following() {    // once turning ended call this to nullify integration -> switch to following
        integrator_.setCalibration(gyro_samples_);
        gyro_samples_.clear();
        state_ = CORRIDOR_FOLLOWING;
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Switched to CORRIDOR_FOLLOWING");
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {     // data from IMU for numerous purposes
        float gyro_z = msg->angular_velocity.z;
        double now = this->now().seconds();
        static double last_time = now;
        double dt = now - last_time;
        last_time = now;
        if (dt <= 0) return;

        if (state_ == CALIBRATION) {    // calibration has priority for data
            gyro_samples_.push_back(gyro_z);
        } else if (state_ == TURNING) {
            integrator_.update(gyro_z, dt);
            float delta_yaw = integrator_.getYaw() - yaw_start_;
            RCLCPP_INFO(this->get_logger(), "Turning... Delta Yaw: %f", delta_yaw);

            if (abs(turn_direction_) > 0 && (std::fabs(delta_yaw)) >= M_PI / 3) {     // end of turning after pi/3 -> 60°? should it be 90°? s pi/4 funguje otaceni o 90
                integrator_.reset();
                if(state_ != STOP){  // added 17.4.
                    state_ = GOING_STRAIGHT_AFTER_TURN;
                    go_straight();
                    RCLCPP_INFO(this->get_logger(), "Finished turn 90, now GOING_STRAIGHT_AFTER_TURN");
                }
            }else if (turn_direction_ == 0 && (std::fabs(delta_yaw)) >= M_PI/1.2) {
                if(state_ != STOP){  // added 17.4.
                    state_ = GOING_STRAIGHT_AFTER_TURN;
                    go_straight();
                    RCLCPP_INFO(this->get_logger(), "Finished turn 180, now GOING_STRAIGHT_AFTER_TURN");
                }
            } else {
                rotate_in_place(turn_direction_);
            }
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {     // from lidar data to offset value for motors
        if (state_ != CORRIDOR_FOLLOWING) return;

        algorithms::LidarFiltr filter;
        auto results = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
        bool need_ = true;

        if (need_ && results.left > 0.35f && results.front < 0.25f) {    // deciding which way to turn
            start_turning(1);                                          // 1 == turn left
            return;
        } else if (need_ && results.right > 0.35f && results.front < 0.25f) {    // old 0.3f, 0.3f
            start_turning(-1);
            return;
        } else if (need_ && results.front < 0.20f) {    // if too close to wall, turn 180
            start_turning(0);
            return;
        }

        if (results.left > 0.3f) {      // when value > 30 cm detected overwrite with 25 cm
            results.left = 0.25f;
            //need_ = true;
        }
        if (results.right > 0.3f) {
            results.right = 0.25f;
            //need_ = true;
        }
    /*
        if (need_ && results.right < 0.3f && results.front < 0.3f) {    // deciding which way to turn
            start_turning(-1);                                          // -1 == turn left??
            return;
        } else if (need_ && results.left < 0.3f && results.front < 0.3f) {
            start_turning(1);
            return;
        }
        */
        float center_offset = 0.0f;
        if (results.left > 0 && results.right > 0) {     // normal "in center of corridor" state
            center_offset = ((results.right - results.left) / 2) * 100;
        } else if (results.right > 0) {     // too close to left wall
            center_offset = -20;
        } else if (results.left > 0) {      // too close to right wall
            center_offset = 20;
        }

        algorithms::Pid pid(0.30, 0.02, 0.08); //0.08d
        float x = pid.step(center_offset, 2);
        algorithms::Pid pid2(0.30, 0.0, 0.0); //0.08d
        float y = pid2.step(x, 1.2);
        send_motor_command(x-y);
    }

    void start_turning(int direction) {
        yaw_start_ = integrator_.getYaw();
        turn_direction_ = direction;
        state_ = TURNING;
        RCLCPP_INFO(this->get_logger(), "Starting turn to %s", direction > 0 ? "right" : "left");
    }

    void go_straight() {
        go_straight_start_time_ = this->now();
        go_straight_timer_ = this->create_wall_timer(
            100ms,
            [this]() {
                auto now = this->now();
                if ((now - go_straight_start_time_).seconds() >= 1.8) {
                    go_straight_timer_->cancel();
                    if (state_ == GOING_STRAIGHT_AFTER_TURN && state_ != STOP) { // stop added 17.4.
                        state_ = CORRIDOR_FOLLOWING;
                        RCLCPP_INFO(this->get_logger(), "Straight movement complete. Back to CORRIDOR_FOLLOWING.");
                    }
                    return;
                }
                std_msgs::msg::UInt8MultiArray msg;
                msg.data = {140, 140};
                motor_pub_->publish(msg);
            });
    }

    void rotate_in_place(int direction) {
        std_msgs::msg::UInt8MultiArray msg;
        uint8_t base = 128;         // 130 is base motor speed +-20 is 110 and 150
        if (direction == 0)
            msg.data = {static_cast<uint8_t>(base - 12), static_cast<uint8_t>(base + 12)};
        else
            msg.data = {static_cast<uint8_t>(base - 12 * direction), static_cast<uint8_t>(base + 12 * direction)};
        motor_pub_->publish(msg);
    }

    void send_motor_command(float pid_output) {
        uint8_t base_speed = 135;
        std_msgs::msg::UInt8MultiArray msg;
        if (pid_output > 0) {
            msg.data = {base_speed, static_cast<uint8_t>(base_speed - pid_output)};
        } else {
            msg.data = {static_cast<uint8_t>(base_speed + pid_output), base_speed};
        }
        motor_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    algorithms::PlanarImuIntegrator integrator_;
    std::vector<float> gyro_samples_;

    State state_;
    float yaw_start_;
    int turn_direction_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorRobot>());
    rclcpp::shutdown();
    return 0;
}