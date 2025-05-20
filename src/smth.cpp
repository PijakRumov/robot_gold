#include <cmath>
#include <vector>
#include <numeric>
#include <chrono>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "pid.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "camera_node.cpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

rclcpp::TimerBase::SharedPtr go_straight_timer_;
rclcpp::Time go_straight_start_time_;
rclcpp::Time pause_and_turn_start_;
rclcpp::TimerBase::SharedPtr pause_and_turn_timer_;

template<typename T>
class MedianFilter {
private:
    std::vector<T> buffer;
    size_t maxSize = 5;
    size_t index = 0;

public:
    MedianFilter() : buffer(5, T()) {}

    void addValue(T value) {
        buffer[index] = value;
        index = (index + 1) % maxSize;
    }

    T getMedian() const {
        std::vector<T> temp = buffer;
        std::sort(temp.begin(), temp.end());
        return temp[maxSize / 2];
    }
};

namespace algorithms {

struct PolarPoint {
    float angle;     // in radians
    float distance;  // in meters or relevant unit
};
 
struct LidarFiltrResults {
    float front;
    float back;
    float left;
    float right;
    float left_front_corner;
    float right_front_corner;
    float left_back_corner;
    float right_back_corner;
    std::pair<float, float> line_right;
    std::pair<float, float> line_left;
    std::pair<float, float> line_right_back;
    std::pair<float, float> line_left_back;
    float median_right;
};

class LidarFiltr {
public:
    LidarFiltrResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
        constexpr float angle_range = M_PI / 12; //old = pi/12 => 15°; sensing 2x15° from all directions // ZK1 melo PI/
        std::vector<float> left, right, front, back, left_front_corner, right_front_corner, left_back_corner, right_back_corner;
        std::vector<PolarPoint> line_right, line_left, line_right_back, line_left_back;
        auto angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            auto angle = angle_start + i * angle_step;
            float dist = points[i];
            if (std::isinf(points[i])) continue;

            if (angle >= -angle_range && angle <= angle_range)  // vector of distances for each of 4 directions
                back.push_back(points[i]);
            else if (angle > M_PI / 2 - angle_range && angle <= M_PI / 2 + angle_range)
                right.push_back(points[i]);
            else if (angle > -M_PI / 2 - angle_range && angle <= -M_PI / 2 + angle_range)
                left.push_back(points[i]);
            else if (angle < -M_PI + angle_range || angle > M_PI - angle_range)
                front.push_back(points[i]);

            else if (angle > M_PI / 2 + M_PI / 6 && angle <= M_PI * (17/24)){ // old M_PI / 2 + M_PI / 4; merim mensi uhel
                line_right.push_back({angle, dist});
            }
            else if (angle < -M_PI / 2 - M_PI / 6 && angle >= -M_PI * (17/24)){ // old -M_PI / 2 - M_PI / 4
                line_left.push_back({angle, dist});
            }
            else if (angle < M_PI / 2 - M_PI / 6 && angle >= M_PI * (7/24)){ // old M_PI / 2 - M_PI / 4)
                line_right_back.push_back({angle, dist});
            }
            else if (angle > -M_PI / 2 + M_PI / 6 && angle <= -M_PI * (7/24)){ // old -M_PI / 2 + M_PI / 4
                line_left_back.push_back({angle, dist});
            }           
            else if (angle > M_PI / 4 - angle_range && angle <= M_PI / 4 + angle_range) {
                right_back_corner.push_back(points[i]);
            }
            else if (angle > -M_PI / 4 - angle_range && angle <= -M_PI / 4 + angle_range) {
                left_back_corner.push_back(points[i]);
            }
            else if (angle > 3 * M_PI / 4 - angle_range && angle <= 3* M_PI / 4 + angle_range) {
                right_front_corner.push_back(points[i]);
            }
            else if (angle > -3 * M_PI / 4 -angle_range && angle <= -3 * M_PI / 4 + angle_range) {
                left_front_corner.push_back(points[i]);
            }        
                
            else{
                // what U doin?
                // nothin, just hangin around
                // dopsano 29.4.
            }
            }

        return {    // sum and divide by number of values -> average value returned
            .front = front.empty() ? 0.0f : std::accumulate(front.begin(), front.end(), 0.0f) / front.size(),
            .back = back.empty() ? 0.0f : std::accumulate(back.begin(), back.end(), 0.0f) / back.size(),
            .left = left.empty() ? 0.0f : std::accumulate(left.begin(), left.end(), 0.0f) / left.size(),
            .right = right.empty() ? 0.0f : std::accumulate(right.begin(), right.end(), 0.0f) / right.size(),
            .left_front_corner = left_front_corner.empty() ? 0.0f : std::accumulate(left_front_corner.begin(), left_front_corner.end(), 0.0f) / left_front_corner.size(),
            .right_front_corner = right_front_corner.empty() ? 0.0f : std::accumulate(right_front_corner.begin(), right_front_corner.end(), 0.0f) / right_front_corner.size(),
            .left_back_corner = left_back_corner.empty() ? 0.0f : std::accumulate(left_back_corner.begin(), left_back_corner.end(), 0.0f) / left_back_corner.size(),
            .right_back_corner = right_back_corner.empty() ? 0.0f : std::accumulate(right_back_corner.begin(), right_back_corner.end(), 0.0f) / right_back_corner.size(),
            .line_right = linearRegressionPolar(line_right),
            .line_left = linearRegressionPolar(line_left),
            .line_right_back = linearRegressionPolar(line_right_back),
            .line_left_back = linearRegressionPolar(line_left_back)
            //.median_right = getRightMedian(right)
        };
    }
/*
    float getRightMedian(const std::vector<float>& input) {
        std::vector<float> temp = input;
        std::sort(temp.begin(), temp.end());
        int size = temp.size();
    
        float right_median;
        if (size % 2 == 1) {
            right_median = temp[size / 2];
        } else {
            right_median = (temp[size / 2 - 1] + temp[size / 2]) / 2.0f;
        }
    
        return right_median;
    }
*/
    std::pair<float, float> linearRegressionPolar(const std::vector<PolarPoint>& points) {
        int n = points.size();
        if (n < 2) {
            return {0.0f, 0.0f}; // Not enough points
        }
    
        float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_x2 = 0.0f;
    
        for (const auto& pt : points) {
            float x = pt.distance * std::cos(pt.angle);
            float y = pt.distance * std::sin(pt.angle);
    
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
    
        float denom = n * sum_x2 - sum_x * sum_x;
        if (denom == 0.0f) return {0.0f, 0.0f};
    
        float a = (n * sum_xy - sum_x * sum_y) / denom;
        float b = (sum_y * sum_x2 - sum_x * sum_xy) / denom;
    
        return {a, b}; // y = a * angle + b
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
    enum State { CALIBRATION, CORRIDOR_FOLLOWING, TURNING, STRAIGHT_DELAY, GOING_STRAIGHT_AFTER_TURN, STOP, GOING_TO_CENTRE, PAUSE_AND_TURN};
    int turn_to_ = -5;  // persistent across callbacks
    int detected_corridor = 0;  // 0 = no corridor, 1 = left front, 2 = right front, 3 = left right, 4 = right front MK, 5 = left front MK, 6 = crossroads, 7 = only right, 8 = only left

    struct LaneLineData {
        float front;
        float right;
        float back;
        float right_front_corner;
        float right_back_corner;
        float left;
        float left_front_corner;
        float left_back_corner;
        float rightA;
        float rightB;
        float leftA;
        float leftB;
        float rightA_back;
        float rightB_back;
        float leftA_back;
        float leftB_back;
    };
    LaneLineData laneData;

    CorridorRobot(std::shared_ptr<CameraNode> camera_node) : camera_node_(camera_node), Node("corridor_robot"), state_(CALIBRATION), yaw_start_(0.0) {
        motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/bpc_prp_robot/lidar", 10, std::bind(&CorridorRobot::lidar_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/bpc_prp_robot/imu", 10, std::bind(&CorridorRobot::imu_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::seconds(8), std::bind(&CorridorRobot::switch_to_following, this));

        pid = std::make_shared<algorithms::Pid>(0.3, 0.0, 0.08);  // Parametry pro regulátor pro střed koridoru
        pid2 = std::make_shared<algorithms::Pid>(0.30, 0.0, 0.08); 
        RCLCPP_INFO(this->get_logger(), "CorridorRobot initialized");
    }

private:
    std::shared_ptr<CameraNode> camera_node_;
    MedianFilter<float> lineRightSlopeFilter;
    MedianFilter<float> lineRightInterceptFilter;
    MedianFilter<float> lineLeftSlopeFilter;
    MedianFilter<float> lineLeftInterceptFilter;
    MedianFilter<float> lineRightSlopeFilterBack;
    MedianFilter<float> lineRightInterceptFilterBack;
    MedianFilter<float> lineLeftSlopeFilterBack;
    MedianFilter<float> lineLeftInterceptFilterBack;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::TimerBase::SharedPtr pause_and_turn_timer_;

    std::shared_ptr<algorithms::Pid> pid;
    std::shared_ptr<algorithms::Pid> pid2;


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
            check_for_marker(camera_node_->getLastMarkerId());      // cteni markeru pri otaceni
            RCLCPP_INFO(this->get_logger(), "Turning... Delta Yaw: %f", delta_yaw);


                if (abs(turn_direction_) > 0 && (std::fabs(delta_yaw)) >= M_PI / 2.4) {     // end of turning after pi/3 -> 60°? should it be 90°? s pi/4 funguje otaceni o 90
                    integrator_.reset();
                    if(state_ != STOP){  // added 17.4.
                        state_ = GOING_STRAIGHT_AFTER_TURN;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(1200));
                        go_straight(0.4);
                        RCLCPP_INFO(this->get_logger(), "Finished turn 90, now GOING_STRAIGHT_AFTER_TURN");
                    }
                }else if (turn_direction_ == 0 && (std::fabs(delta_yaw)) >= M_PI/1.1) {
                    if(state_ != STOP){  // added 17.4.
                        state_ = GOING_STRAIGHT_AFTER_TURN;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(1200));
                        go_straight(0.4);
                        RCLCPP_INFO(this->get_logger(), "Finished turn 180, now GOING_STRAIGHT_AFTER_TURN");
                    }
                } else {
                    rotate_in_place(turn_direction_);
                }
        }
    }

    void check_for_marker(int marker_id) {     // check for ArUco marker ID
        if (turn_to_ != -5 || marker_id == turn_to_) 
            return;  // already got it

        if (marker_id != -5) {
            turn_to_ = marker_id;

            if (turn_to_ == 10) {
                turn_to_ = -5;  // right
            } else if (turn_to_ == 11) {
                turn_to_ = 1;  // left
            } else if (turn_to_ == 12) {
                turn_to_ = 2;  // straight
            }
            RCLCPP_INFO(this->get_logger(), "Stored marker ID: %d", turn_to_);
        }
    }

    float getRightMedian(const std::vector<float>& input) {
        std::vector<float> temp = input;
        std::sort(temp.begin(), temp.end());
        int size = temp.size();
    
        float right_median;
        if (size % 2 == 1) {
            right_median = temp[size / 2];
        } else {
            right_median = (temp[size / 2 - 1] + temp[size / 2]) / 2.0f;
        }
    
        return right_median;
    }

    int check_for_crossroads(const LaneLineData data){
        int the_a_limit = 1.5;
        check_for_marker(camera_node_->getLastMarkerId());      // cteni markeru pri otaceni
        RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);
        //int detected_corridor = 0;  // 0 = no corridor, 1 = left, 2 = right, 3 = both

        if (((std::abs(data.rightA) > the_a_limit) && (abs(data.rightA_back) > the_a_limit) && std::abs(data.rightB) < 2) &&  ((std::abs(data.leftA) > the_a_limit) && (std::abs(data.leftA_back) > the_a_limit) && std::abs(data.leftB) < 2) && data.front > 0.35f) {
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t RIGHT LEFT \t by LINES");
            detected_corridor = 3;
            return 1;
        } else if ((std::abs(data.rightA) > the_a_limit) && std::abs(data.rightA) > the_a_limit && std::abs(data.rightB) < 2 && (std::abs(data.rightA_back) > the_a_limit) && data.front > 0.40f) {
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t RIGHT FRONT \t by LINES");
            detected_corridor = 2;
            //RCLCPP_INFO(this->get_logger(), "RIGHT line: y = ax + b; a= %f, b= %f", smoothedRightA, smoothedRightB); // info text
            return 1;
        } else if ((std::abs(data.leftA) > the_a_limit) && std::abs(data.leftB) > the_a_limit && std::abs(data.leftB) < 2 && (std::abs(data.leftA_back) > the_a_limit) && data.front > 0.40f) {
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t LEFT FRONT \t by LINES");
            detected_corridor = 1;
            //RCLCPP_INFO(this->get_logger(), "LEFT line: y = ax + b; a= %f, b= %f", smoothedLeftA, smoothedLeftB); // info text
            return 1;
        } else if (data.right > 0.35f && data.right_front_corner > 0.35f && data.left_front_corner > 0.35f && data.front > 0.45f && data.left > 0.35f && data.back > 0.45f){
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t RIGHT LEFT FRONT \t by MK");
            detected_corridor = 6;
            return 1;
        } else if (data.front < 0.27f && data.right > 0.35f && data.left > 0.35f && data.back > 0.45f){
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t RIGHT LEFT \t by MK");
            detected_corridor = 3; // not ideal, could start turning at wrong spots
            return 1;
        } else if (data.right > 0.35f && data.right_front_corner > 0.35f && data.front > 0.45f && data.back > 0.45f){
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t RIGHT FRONT \t by MK");
            RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);
            detected_corridor = 4;
            return 1;
        } else if (data.left > 0.35f && data.left_front_corner > 0.35f && data.front > 0.45f && data.back > 0.45f){
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t on LEFT FRONT \t by MK");
            detected_corridor = 5;   
            return 1;
        //} else if (data.front > Xf && data.right/left > Xf){     // lets hope we wont use this
        //    RCLCPP_INFO(this->get_logger(), "crossroads detected \t XXXX \t by BACKBONE");
        //    detected_corridor = X;
        //    return 1;   
        } else {
            RCLCPP_INFO(this->get_logger(), "crossroads detected \t NOT");
            //RCLCPP_INFO(this->get_logger(), "right: \t %f \t right_front: %f", data.right, data.right_front_corner);

            //RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);   // info text
            detected_corridor = 0;
            //RCLCPP_INFO(this->get_logger(), "RIGHT median: %f", results.median_right); // info text
            //RCLCPP_INFO(this->get_logger(), "LEFT line: y = ax + b; a= %f, b= %f", smoothedLeftA, smoothedLeftB); // info text
            //RCLCPP_INFO(this->get_logger(), "RIGHT line: y = ax + b; a= %f, b= %f", data.rightA, data.rightB); // info text
            //RCLCPP_INFO(this->get_logger(), "LEFT line BACK: y = ax + b; a= %f, b= %f", smoothedLeftA_back, smoothedLeftB_back); // info text
            //RCLCPP_INFO(this->get_logger(), "RIGHT line BACK: y = ax + b; a= %f, b= %f", data.rightA_back, data.rightB_back); // info text
            return 0;
        }
        //return 0;
    }

    int check_for_corridor(const LaneLineData data){        // values not used furtherore, but can be
        //int detected_corridor = 0;  // 0 = no corridor, 1 = left, 2 = right, 3 = both

        if (data.right > 0.35f && data.front < 0.30f){
            RCLCPP_INFO(this->get_logger(), "corridor detected \t RIGHT \t by MK");
            detected_corridor = 7;
            return 1;
        } else if (data.left > 0.35f && data.front < 0.30f){
            RCLCPP_INFO(this->get_logger(), "corridor detected \t on LEFT \t by MK");
            detected_corridor = 8;   
            return 1;    
        } else {
            RCLCPP_INFO(this->get_logger(), "corridor detected \t NOT");
            RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);
            detected_corridor = 0;
            return 0;
        }
        //return 0;
    }

    void turning_decision(const algorithms::LidarFiltrResults& results) {
        int last_marker_id = camera_node_->getLastMarkerId();
        check_for_marker(last_marker_id);
        RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);
        RCLCPP_INFO(this->get_logger(), "detected corridor num: %d", detected_corridor);
        if (turn_to_!= -5){       // kontrola jestli je na stacku marker
            if(turn_to_ > 0 && turn_to_ < 7 && results.front > 0.25){ // reaguje na crossroads
                //RCLCPP_INFO(this->get_logger(), "Detected ArUco marker: %d", turn_to_);
                if((turn_to_ == 1) && (detected_corridor == 5 || detected_corridor == 6)){
                    RCLCPP_INFO(this->get_logger(), "pause and turning \t LEFT \t by MK");
                    state_ = PAUSE_AND_TURN;
                    pause_and_turn(1); // 1 = turn LEFT
                    //start_turning(1); // 1 = turn LEFT
                    //turn_to_ = -5; // reset marker ID
                    return;
                } else if ((turn_to_ == 2) && (detected_corridor == 4 || detected_corridor == 6)) {    // deciding which way to turn
                    RCLCPP_INFO(this->get_logger(), "pause and turning \t RIGHT \t by MK");
                    state_ = PAUSE_AND_TURN;
                    pause_and_turn(-1); // -1 == turn RIGHT
                    //start_turning(-1);                                          // -1 == turn RIGHT
                    //turn_to_ = -5; // reset marker ID
                    return;
                } else if ((turn_to_ == 1) && (detected_corridor == 1 || detected_corridor == 3 )) {
                    RCLCPP_INFO(this->get_logger(), "turning \t LEFT \t by LINES");
                    start_turning(1); // 1 = turn LEFT
                    turn_to_ = -5;
                    return;
                } else if ((turn_to_ == 2) && (detected_corridor == 2 || detected_corridor == 3)) {
                    RCLCPP_INFO(this->get_logger(), "turning \t RIGHT \t by LINES");
                    start_turning(-1); // -1 == turn RIGHT
                    turn_to_ = -5;
                    return;
                } else if ((turn_to_ == 0) && (detected_corridor == 1 || detected_corridor == 2 || detected_corridor == 4 || detected_corridor == 5 || detected_corridor == 6)) {
                    RCLCPP_INFO(this->get_logger(), "turning \t STRAIGHT \t by YO MAMA");
                    turn_to_ = -5;
                    return;
                }
            }else{      // executed when only corridor detected
                if (results.left > 0.35f && results.front < 0.25f) {    // old < 0.19f
                    start_turning(1);                                          // 1 == turn left
                    return;
                } else if (results.right > 0.35f && results.front < 0.25f) {    // old 0.3f, 0.3f
                    start_turning(-1);
                    return;
                } else if (results.front < 0.20f) {    // if too close to wall, turn 180
                    start_turning(0);
                    return;
                }

            }
        }else{

            if (results.left > 0.35f && results.front < 0.25f) {    // deciding which way to turn
                start_turning(1);                                          // 1 == turn left
                return;
            } else if (results.right > 0.35f && results.front < 0.25f) {    // old 0.3f, 0.3f
                start_turning(-1);
                return;
            } else if (results.front < 0.20f) {    // if too close to wall, turn 180
                start_turning(0);
                return;
            }
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {     // from lidar data to offset value for motors
        if (state_ != CORRIDOR_FOLLOWING) return;


        algorithms::LidarFiltr filter;
        auto results = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max);
        bool need_ = true;
        int last_marker_id = camera_node_->getLastMarkerId();
        check_for_marker(last_marker_id);
        
        //int detected_corridor = 0;  // 0 = no corridor, 1 = left, 2 = right, 3 = both

        
        lineRightSlopeFilter.addValue(results.line_right.first);
        lineRightInterceptFilter.addValue(results.line_right.second);
        lineLeftSlopeFilter.addValue(results.line_left.first);
        lineLeftInterceptFilter.addValue(results.line_left.second);
        lineRightSlopeFilterBack.addValue(results.line_right_back.first);
        lineRightInterceptFilterBack.addValue(results.line_right_back.second);
        lineLeftSlopeFilterBack.addValue(results.line_left_back.first);
        lineLeftInterceptFilterBack.addValue(results.line_left_back.second);

        laneData.rightA = lineRightSlopeFilter.getMedian();
        laneData.rightB = lineRightInterceptFilter.getMedian();
        laneData.leftA = lineLeftSlopeFilter.getMedian();
        laneData.leftB = lineLeftInterceptFilter.getMedian();
        laneData.rightA_back = lineRightSlopeFilterBack.getMedian();
        laneData.rightB_back = lineRightInterceptFilterBack.getMedian();
        laneData.leftA_back = lineLeftSlopeFilterBack.getMedian();
        laneData.leftB_back = lineLeftInterceptFilterBack.getMedian();
        laneData.front = results.front;
        laneData.right = results.right;
        laneData.left = results.left;
        laneData.right_front_corner = results.right_front_corner;
        laneData.left_front_corner = results.left_front_corner;
        laneData.right_back_corner = results.right_back_corner;
        laneData.left_back_corner = results.left_back_corner;
        laneData.back = results.back;
        /*
        if(check_for_crossroads(laneData) != 1){
            if(check_for_corridor(laneData) != 1){
                //everythin is fucked or no corridor detected
            }
        }
            */
        int pokus = check_for_crossroads(laneData);
        //int pokuss = check_for_corridor(laneData); 

        turning_decision(results);
        // the old way
        //if (((smoothedRightA <= -5 || smoothedRightA >= 5) && std::abs(smoothedRightB) > 0.6 && std::abs(smoothedRightB) < 2) &&  ((smoothedLeftA <= -5 || smoothedLeftA >= 5) && std::abs(smoothedLeftB) > 0.6 && std::abs(smoothedLeftB) < 2)) {
        /*
        int the_a_limit = 1.5;
        

        if (((std::abs(smoothedRightA) > the_a_limit) && (abs(smoothedRightA_back) > the_a_limit) && std::abs(smoothedRightB) < 2) &&  ((std::abs(smoothedLeftA) > the_a_limit) && (std::abs(smoothedLeftA_back) > the_a_limit) && std::abs(smoothedLeftB) < 2)) {
            RCLCPP_INFO(this->get_logger(), "corridor detected on BOTH sides");
            detected_corridor = 3;
        } else if ((std::abs(smoothedRightA_back) > the_a_limit) && std::abs(smoothedRightB_back) > 0.6 && std::abs(smoothedRightB_back) < 2 && results.right > 0.30f){// && && std::abs(results.median_right) > 1.1
            RCLCPP_INFO(this->get_logger(), "corridor detected on RIGHT");
            detected_corridor = 2;
            //RCLCPP_INFO(this->get_logger(), "RIGHT line: y = ax + b; a= %f, b= %f", smoothedRightA, smoothedRightB); // info text
        } else if ((std::abs(smoothedLeftA) > the_a_limit) && std::abs(smoothedLeftB) > 0.6 && std::abs(smoothedLeftB) < 2 && (std::abs(smoothedLeftA_back) > the_a_limit)) {
            RCLCPP_INFO(this->get_logger(), "corridor detected on LEFT");
            detected_corridor = 1;
            //RCLCPP_INFO(this->get_logger(), "LEFT line: y = ax + b; a= %f, b= %f", smoothedLeftA, smoothedLeftB); // info text
        } else {
            RCLCPP_INFO(this->get_logger(), "corridor NOT detected");
            RCLCPP_INFO(this->get_logger(), "last ArUco marker: %d", turn_to_);   // info text
            detected_corridor = 0;
            RCLCPP_INFO(this->get_logger(), "RIGHT median: %f", results.median_right); // info text
            //RCLCPP_INFO(this->get_logger(), "LEFT line: y = ax + b; a= %f, b= %f", smoothedLeftA, smoothedLeftB); // info text
            RCLCPP_INFO(this->get_logger(), "RIGHT line: y = ax + b; a= %f, b= %f", data.rightA, data.rightB); // info text
            //RCLCPP_INFO(this->get_logger(), "LEFT line BACK: y = ax + b; a= %f, b= %f", smoothedLeftA_back, smoothedLeftB_back); // info text
            RCLCPP_INFO(this->get_logger(), "RIGHT line BACK: y = ax + b; a= %f, b= %f", data.rightA_back, data.rightB_back); // info text
        
        }
            */
        /*
        if (turn_to_!= -5){        // NEJSME schopni projet zatacku + !!!!!! && results.front > 0.27f
            if(turn_to_ == 0 || turn_to_ == 1 || turn_to_ == 2){ // marker ID 0, 1, 2
                RCLCPP_INFO(this->get_logger(), "Detected ArUco marker: %d", turn_to_);
                if((turn_to_ == 1) && (need_ == true) && (detected_corridor == 1 || detected_corridor == 3)){//(detected_corridor == 1 || detected_corridor == 3)) {    // if too close to wall, turn 180
                    left_line_end = 0;
                    start_turning(1);
                    turn_to_ = -5; // reset marker ID
                    return;
                } else if ((turn_to_ == 2) && ((need_ == true) && (detected_corridor == 2 || detected_corridor == 3) || right_line_end == 1)) {    // deciding which way to turn
                    right_line_end = 0;
                    start_turning(-1);                                          // -1 == turn left
                    turn_to_ = -5; // reset marker ID
                    return;
                } else if ((turn_to_ == 0)) {
                    turn_to_ = -5;
                    return;
                } else if (need_ && results.left > 0.35f && results.front < 0.19f) {    // deciding which way to turn
                    start_turning(1);                                          // 1 == turn left
                    return;
                } else if (need_ && results.right > 0.35f && results.front < 0.19f) {    // old 0.3f, 0.3f
                    start_turning(-1);
                    return;
                } else if (need_ && results.front < 0.16f) {    // if too close to wall, turn 180
                    start_turning(0);
                    return;
                }
            }else{
                //RCLCPP_INFO(this->get_logger(), "No ArUco marker detected"); // info text

            }
        }else{

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
        }
            */
        if (results.left > 0.3f) {      // when value > 30 cm detected overwrite with 25 cm
            results.left = 0.21f;
            //need_ = true;
        }
        if (results.right > 0.3f) {
            results.right = 0.21f;  // old 0.25f
            //need_ = true;
        }
        if(results.left_front_corner > 0.3f){
            results.left_front_corner = 0.25f;
        }
        if(results.right_front_corner > 0.3f){
            results.right_front_corner = 0.25f;
        }


        float center_offset = 0.0f;
        if (results.left > 0 && results.right > 0) {     // normal "in center of corridor" state
            center_offset = ((results.right + results.right_front_corner) /2  - 
                             (results.left + results.left_front_corner) / 2) * 100;
            RCLCPP_INFO(this->get_logger(), "center offset TRUE: %f", center_offset);
        } else if (results.right > 0) {     // too close to left wall
            center_offset = -10;
        } else if (results.left > 0) {      // too close to right wall
            center_offset = 10;
        }

        

        if (std::fabs(center_offset) < 10.0f) {    // if in center of corridor
            float x = pid->step(center_offset, 0.1);
            RCLCPP_INFO(this->get_logger(), "center offset PID: %f", center_offset);
            //RCLCPP_INFO(this->get_logger(), "center offset: %f", center_offset);
            //float y = pid2->step(x, 0.1);
            //RCLCPP_INFO(this->get_logger(), "PID output: %f", x);
            send_motor_command(x);
        }else if (center_offset > 10.0f){
            center_offset = 10.0f;
            RCLCPP_INFO(this->get_logger(), "center offset PID: %f", center_offset);
            float x = pid->step(center_offset, 0.1);
            //RCLCPP_INFO(this->get_logger(), "center offset: %f", center_offset);
            //float y = pid2->step(x, 0.1);
            //RCLCPP_INFO(this->get_logger(), "PID output: %f", x);
            send_motor_command(x);
        }else if (center_offset < -10.0f){
            center_offset = -10.0f;
            float x = pid->step(center_offset, 0.1);
            RCLCPP_INFO(this->get_logger(), "center offset: %f", center_offset);
            //float y = pid2->step(x, 0.1);
            //RCLCPP_INFO(this->get_logger(), "PID output: %f", x);
            send_motor_command(x);
        }else {
            std_msgs::msg::UInt8MultiArray msg;
            msg.data = {145, 145};  
            motor_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Motors running...");
        }
    }
    /*
    void pause_and_turn(int direction, const LaneLineData data) {
    RCLCPP_INFO(this->get_logger(), "Going straight for 1 second before turning");
    pause_and_turn_start_ = this->now();
    pause_and_turn_timer_ = this->create_wall_timer(
        100ms,
        [this, direction]() {
            auto now = this->now();
            if ((now - pause_and_turn_start_).seconds() >= 1.6) {// 1.6 je doba timeru ve vterinach
                pause_and_turn_timer_->cancel();
                if(direction == 1 && data.left > 0.35f && data.left_front_corner > 0.35f){
                    RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning LEFT");
                    start_turning(direction);
                } else if (direction == -1 && data.right > 0.35f && data.right_front_corner > 0.35f){
                    RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning RIGHT");
                    start_turning(direction);
                }
                RCLCPP_INFO(this->get_logger(), "Straight after pause complete. crossroads NOT detected");
                //start_turning(direction);
                return;
            } else {
                std_msgs::msg::UInt8MultiArray msg;
                msg.data = {140, 140};
                motor_pub_->publish(msg);
            }
        }
    );
    }
    */
    void pause_and_turn(int direction){//, const LaneLineData data) {
        RCLCPP_INFO(this->get_logger(), "Going straight for 1 second before turning");
        pause_and_turn_start_ = this->now();
        pause_and_turn_timer_ = this->create_wall_timer(
           100ms,
           [this, direction]() {
                auto now = this->now();
                if ((now - pause_and_turn_start_).seconds() >= 0.6) {
                    pause_and_turn_timer_->cancel();
                    /*
                    RCLCPP_INFO(this->get_logger(), "Straight after pause complete. turning");
                    start_turning(direction);
                    */
                    if(direction == 1){
                        RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning LEFT");
                        start_turning(direction);
                        turn_to_ = -5;
                    } else if (direction == -1){
                        RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning RIGHT");
                        start_turning(direction);
                        turn_to_ = -5;
                    } else{
                    RCLCPP_INFO(this->get_logger(), "Straight after pause complete. crossroads NOT detected");
                        state_ = CORRIDOR_FOLLOWING;
                    }
                    /*
                    if(direction == 1 && data.left > 0.35f && data.left_front_corner > 0.35f && data.left_back_corner > 0.35f){
                        RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning LEFT");
                        start_turning(direction);
                    } else if (direction == -1 && data.right > 0.35f && data.right_front_corner > 0.35f && data.right_back_corner > 0.35f){
                        RCLCPP_INFO(this->get_logger(), "Straight after pause complete. Now Turning RIGHT");
                        start_turning(direction);
                    }
                    RCLCPP_INFO(this->get_logger(), "Straight after pause complete. crossroads NOT detected");
                    */
                    //state_ = CORRIDOR_FOLLOWING;
                    return;
                } else {
                    std_msgs::msg::UInt8MultiArray msg;
                    msg.data = {140, 140};
                    motor_pub_->publish(msg);
                }
            }
        );
    }


    void start_turning(int direction) {
        yaw_start_ = integrator_.getYaw();
        turn_direction_ = direction;
        state_ = TURNING;
        RCLCPP_INFO(this->get_logger(), "Starting turn to %s", direction > 0 ? "left" : "right");
    }

    void go_straight(double duration_seconds) {
        go_straight_start_time_ = this->now();
        go_straight_timer_ = this->create_wall_timer(
            100ms,
            [this, duration_seconds]() {
                auto now = this->now();
                int last_marker_id = camera_node_->getLastMarkerId();
                check_for_marker(last_marker_id);
                if ((now - go_straight_start_time_).seconds() >= duration_seconds) {
                    go_straight_timer_->cancel();
                    if (state_ == GOING_STRAIGHT_AFTER_TURN) { // stop added 17.4.
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
        uint8_t base_speed = 150;
        std_msgs::msg::UInt8MultiArray msg;
        if (pid_output > 0) {
            msg.data = {base_speed, static_cast<uint8_t>(base_speed - pid_output)};
        } else {
            msg.data = {static_cast<uint8_t>(base_speed + pid_output), base_speed};
        }
        motor_pub_->publish(msg);
        //pid->reset();
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

    auto camera_node = std::make_shared<CameraNode>();
    auto corridor_robot_node = std::make_shared<CorridorRobot>(camera_node);

    rclcpp::executors::MultiThreadedExecutor executor;  // old SinfleThreadedExecutor
    executor.add_node(corridor_robot_node);
    executor.add_node(camera_node);

    executor.spin();
    //rclcpp::spin(std::make_shared<CorridorRobot>());
    rclcpp::shutdown();
    return 0;
}
