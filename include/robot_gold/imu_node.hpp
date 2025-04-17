#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "planar_imu_integrator.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();
        ~ImuNode() override = default;

        // Set the IMU Mode
        void setMode(const ImuNodeMode setMode);

        // Get the current IMU Mode
        ImuNodeMode getMode();

        // Get the results after Integration
        auto getIntegratedResults();

        // Reset the class
        void reset_imu();
        std_msgs::msg::UInt8MultiArray message_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;


    private:

        void calibrate();
        void integrate();

        ImuNodeMode mode = ImuNodeMode::INTEGRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}
