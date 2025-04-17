#include <iostream>
#include"rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_node.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "pid.hpp"

using std::placeholders::_1;

namespace algorithms {

    PlanarImuIntegrator::PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

    void PlanarImuIntegrator::update(float gyro_z, double dt) {
        float corrected_gyro_z = gyro_z - gyro_offset_;
        theta_ += corrected_gyro_z * dt;
    }

    void PlanarImuIntegrator::setCalibration(std::vector<float> gyro) {
        if (gyro.empty()) {
            gyro_offset_ = 0.0f;
            return;
        }

        float sum = std::accumulate(gyro.begin(), gyro.end(), 0.0f);
        gyro_offset_ = sum / static_cast<float>(gyro.size());
    }

    float PlanarImuIntegrator::getYaw() const {
        return theta_;
    }

    void PlanarImuIntegrator::reset() {
        theta_ = 0.0f;
        gyro_offset_ = 0.0f;
    }

}

namespace nodes {

    ImuNode::ImuNode()
    : Node("imu_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10, std::bind(&ImuNode::on_imu_msg, this, _1));
        
        mode = ImuNodeMode::CALIBRATE;
    }

    void ImuNode::setMode(const ImuNodeMode setMode) {
        mode = setMode;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.clear();
        } else if (mode == ImuNodeMode::INTEGRATE) {
            calibrate();
        }
    }

    ImuNodeMode ImuNode::getMode() {
        return mode;
    }

    auto ImuNode::getIntegratedResults() {
        return planar_integrator_.getYaw();
    }

    void ImuNode::reset_imu() {
        planar_integrator_.reset();
        gyro_calibration_samples_.clear();
        mode = ImuNodeMode::CALIBRATE;
    }

    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);
        gyro_calibration_samples_.clear();
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        float gyro_z = msg->angular_velocity.z;
        double current_time = this->now().seconds();

        static double last_time = current_time;
        double dt = current_time - last_time;
        last_time = current_time;

        if (dt <= 0.0) return;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(gyro_z);
        } else if (mode == ImuNodeMode::INTEGRATE) {
            planar_integrator_.update(gyro_z, dt);
        }
        if(mode == ImuNodeMode::INTEGRATE) 
            RCLCPP_INFO(this->get_logger(), "Gyro Z: %f, Yaw: %f", gyro_z, planar_integrator_.getYaw());


        float yaq_erroor = 0;
        if(planar_integrator_.getYaw()> M_PI/36) {
            yaq_erroor = 0 - planar_integrator_.getYaw();
            float correction = 15 * yaq_erroor; // 0.1 je P REGULATOR
            // motor move
            message_.data = {static_cast<uint8_t>(128 - correction), static_cast<uint8_t>(128 + correction)};
            publisher_->publish(message_);
        }else if(planar_integrator_.getYaw() < -M_PI/36) {
            float yaq_erroor = 0 - planar_integrator_.getYaw();
            float correction = 20 * yaq_erroor; // 0.1 je P REGULATOR
            // motor move
            message_.data = {static_cast<uint8_t>(129 - correction), static_cast<uint8_t>(129 + correction)};
            publisher_->publish(message_);
        }
        /*
        if(abs(planar_integrator_.getYaw())> M_PI/36) {
            algorithms::Pid pid(10, 0.5, 0.0);
            float x = pid.step(yaq_erroor, 0.2);
            RCLCPP_INFO(this->get_logger(), "PID output: %f", x); //p 0.4 i 0.2 d 0.0 dt 0.2 funkcni by me
            if(x>0){
                message_.data = {129, static_cast<unsigned char>(129 - x)};
            }
            else {
                message_.data = {static_cast<unsigned char>(129 + x), 129};
            }
            publisher_->publish(message_);
    }
            */
    }

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto imu_node = std::make_shared<nodes::ImuNode>();

    rclcpp::TimerBase::SharedPtr mode_switch_timer;
    mode_switch_timer = imu_node->create_wall_timer(
        std::chrono::seconds(5),
        [imu_node, &mode_switch_timer]() {
            RCLCPP_INFO(imu_node->get_logger(), "Calibration complete. Switching to INTEGRATE mode.");
            imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
            mode_switch_timer->cancel();
        });

    rclcpp::spin(imu_node);
    rclcpp::shutdown();
    return 0;
}
