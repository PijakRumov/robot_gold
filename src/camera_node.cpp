#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "aruco_detector.hpp"

// 0, 10 ---> rovne
// 1, 11 ---> leva
// 2, 12 ---> pravo

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), detector_()
    {
        image_sub_ = image_transport::create_subscription(
            this,
            "/bpc_prp_robot/camera",
            std::bind(&CameraNode::imageCallback, this, std::placeholders::_1),
            "raw");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try
        {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;

            // Store the last frame
            last_frame_ = image.clone();

            // Detect ArUco markers
            last_detections_ = detector_.detect(image);

            // Draw markers and print IDs
            for (const auto &marker : last_detections_)
            {
                cv::aruco::drawDetectedMarkers(image, std::vector<std::vector<cv::Point2f>>{marker.corners}, std::vector<int>{marker.id});
                RCLCPP_INFO(this->get_logger(), "Detected marker ID: %d", marker.id);
            }

            cv::imshow("Camera Image with ArUco", image);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    image_transport::Subscriber image_sub_;
    algorithms::ArucoDetector detector_;

    cv::Mat last_frame_;
    std::vector<algorithms::ArucoDetector::Aruco> last_detections_;
};
/*
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
*/