#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode()
        : Node("aruco_detector_node"),
          dictionary_(
              cv::aruco::getPredefinedDictionary(
                  cv::aruco::DICT_ARUCO_ORIGINAL)),
          detector_params_(),
          detector_(dictionary_, detector_params_)
    {
        subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                10,
                std::bind(
                    &ArucoDetectorNode::image_callback,
                    this,
                    std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_aruco", 10);
        RCLCPP_INFO(
            this->get_logger(),
            "ArUco Detector Node started. Waiting for images...");
    }

    ~ArucoDetectorNode()
    {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;

        detector_.detectMarkers(cv_ptr->image, marker_corners, marker_ids, rejected_candidates);

        if (!marker_ids.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Detected %zu marker(s)", marker_ids.size());

            for (const auto &id : marker_ids)
            {
                RCLCPP_INFO(this->get_logger(), "Marker ID: %d", id);
            }

            cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
        }

        image_pub_->publish(*cv_ptr->toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_params_;
    cv::aruco::ArucoDetector detector_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArucoDetectorNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
