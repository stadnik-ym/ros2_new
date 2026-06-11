#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
    public:
    CameraPublisher() : Node("camera_publisher_node")
    {
        // camera path (/dev/video0 за замовчуванням)
        this->declare_parameter<std::string>("camera_path", "0");
        std::string cam_path = this->get_parameter("camera_path").as_string();
        RCLCPP_INFO(this->get_logger(), "пробуємо відкрити камеру: %s", cam_path.c_str());

        bool is_int = !cam_path.empty() && std::all_of(cam_path.begin(), cam_path.end(), ::isdigit);

        if (is_int)
        {
            cap_.open(std::stoi(cam_path), cv::CAP_V4L2);
        } else
        {
            cap_.open(cam_path);
        }

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "cannot open camera");
            throw std::runtime_error("camera open failed");
        }

        publisher_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        timer_ = create_wall_timer(33ms, std::bind(&CameraPublisher::publish_frame, this));
    }
    private:
    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) return;

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_ -> publish(*msg);
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
