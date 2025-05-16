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

std::vector<std::vector<cv::Point2f>> all_corners;
std::vector<int> all_ids;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), detector_()
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed",
            10,
            std::bind(&CameraNode::imageCallback, this, std::placeholders::_1));
    }
    int getLastMarkerId() const
    {
        if (!last_detections_.empty())
        {
            if(last_detections_.back().id == 0 || last_detections_.back().id == 1 || last_detections_.back().id == 2){
                return -5;      // if odkomentovat pro prioritu modu ESCAPE (TREASURE mod if potrebuje)
            }
            return last_detections_.back().id;
        }
        return -5; // or handle it however you'd like if no markers were detected
    }
private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Decode compressed image using OpenCV
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            if (image.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Decoded image is empty.");
                return;
            }

            // Store the last frame
            last_frame_ = image.clone();

            // Detect ArUco markers
            last_detections_ = detector_.detect(image);

            // Draw markers and print IDs
            for (const auto &marker : last_detections_)
            {
            all_corners.push_back(marker.corners);
            all_ids.push_back(marker.id);
            //RCLCPP_INFO(this->get_logger(), "Detected marker ID: %d", marker.id);
            }

            //cv::aruco::drawDetectedMarkers(image, all_corners, all_ids);

            //cv::imshow("Camera Image with ArUco", image);
            cv::waitKey(1);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
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