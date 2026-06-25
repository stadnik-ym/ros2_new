#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher_node")
    {
        // Параметр оставляем, но теперь он просто включает/выключает libcamera
        this->declare_parameter<std::string>("camera_backend", "libcamera");

        std::string backend = this->get_parameter("camera_backend").as_string();

        RCLCPP_INFO(this->get_logger(),
                    "Starting camera node with backend: %s",
                    backend.c_str());


        std::string pipeline =
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false";

        // if (backend == "libcamera")
        // {
        //     pipeline =
        //         "libcamerasrc ! "
        //         "video/x-raw,format=RGBx,width=640,height=480,framerate=30/1 ! "
        //         "videoconvert ! "
        //         "video/x-raw,format=BGR ! "
        //         "appsink drop=true max-buffers=1 sync=false";
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(),
        //                  "Unsupported backend. Only 'libcamera' is allowed.");
        //     throw std::runtime_error("invalid camera backend");
        // }

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline:\n%s", pipeline.c_str());

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera pipeline");
            throw std::runtime_error("camera open failed");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10);

        timer_ = this->create_wall_timer(
            33ms,
            std::bind(&CameraPublisher::publish_frame, this));

        RCLCPP_INFO(this->get_logger(), "Camera node started successfully");
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame received");
            return;
        }

        auto msg =
            cv_bridge::CvImage(std_msgs::msg::Header(),
                               "bgr8",
                               frame)
                .toImageMsg();

        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";

        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
