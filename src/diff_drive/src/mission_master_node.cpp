#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

class MissionMasterNode : public rclcpp::Node {
public:
    MissionMasterNode()
        : Node("mission_master_node"),
          dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL)),
          detector_params_(),
          detector_(dictionary_, detector_params_),
          target_found_(false),
          marker_x_(0.0),
          marker_y_(0.0)
    {
        // Підписуємся на камеру
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&MissionMasterNode::image_callback, this, std::placeholders::_1));

        // Видаємо команди руху
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Періодично оновлюємо управління (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionMasterNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Mission Master started. Looking for ArUco markers...");
    }

private:
    // Параметри управління
    const double LINEAR_SPEED = 0.15;      // м/с — швидкість їзди до маркера
    const double ANGULAR_SPEED = 0.5;      // рад/с — швидкість повороту
    const double MARKER_DISTANCE_THRESHOLD = 50.0;  // пікселів — допустима відстань до центру
    const double MARKER_ANGLE_THRESHOLD = 0.1;      // радіан — допустимий кут

    // Детекція
    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_params_;
    cv::aruco::ArucoDetector detector_;

    // Стан маркера
    bool target_found_;
    double marker_x_;
    double marker_y_;
    int image_width_;
    int image_height_;

    // ROS 2
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        image_width_ = cv_ptr->image.cols;
        image_height_ = cv_ptr->image.rows;

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;

        detector_.detectMarkers(cv_ptr->image, marker_corners, marker_ids, rejected_candidates);

        if (!marker_ids.empty()) {
            target_found_ = true;

            // Беремо перший (найбільший) маркер
            auto corners = marker_corners[0];

            // Обраховуємо центр маркера
            cv::Point2f center(0, 0);
            for (const auto &corner : corners) {
                center += corner;
            }
            center *= 0.25f;

            marker_x_ = center.x;
            marker_y_ = center.y;

            RCLCPP_DEBUG(
                this->get_logger(),
                "Marker found at (%.1f, %.1f), ID: %d",
                marker_x_, marker_y_, marker_ids[0]);

        } else {
            target_found_ = false;
            RCLCPP_DEBUG(this->get_logger(), "No marker detected");
        }
    }

    void control_loop() {
        auto twist = geometry_msgs::msg::Twist();

        if (!target_found_) {
            // Робот крутиться шукаючи маркер
            RCLCPP_WARN(this->get_logger(), "Marker not found. Searching...");
            twist.angular.z = ANGULAR_SPEED * 0.5;
            cmd_vel_pub_->publish(twist);
            return;
        }

        // Центр зображення
        double center_x = image_width_ / 2.0;
        double center_y = image_height_ / 2.0;

        // Помилка позиції
        double error_x = marker_x_ - center_x;
        double error_y = marker_y_ - center_y;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);

        // Обраховуємо кут на маркер (горизонтально)
        double angle_error = error_x / center_x;  // нормалізована помилка кута

        RCLCPP_INFO(
            this->get_logger(),
            "Distance: %.1f px, Angle error: %.3f rad",
            distance_error, angle_error);

        // Якщо близько до маркера — зупиняємось
        if (distance_error < MARKER_DISTANCE_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(), "Target reached!");
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub_->publish(twist);
            return;
        }

        // Спочатку вирівнюємо кут (чистий поворот)
        if (std::abs(angle_error) > MARKER_ANGLE_THRESHOLD) {
            twist.linear.x = 0.0;
            twist.angular.z = ANGULAR_SPEED * std::clamp(angle_error, -1.0, 1.0);
            RCLCPP_DEBUG(this->get_logger(), "Rotating to marker (angular.z: %.3f)", twist.angular.z);
            cmd_vel_pub_->publish(twist);
            return;
        }

        // Коли вирівнялись — їдемо вперед
        twist.linear.x = LINEAR_SPEED;
        twist.angular.z = ANGULAR_SPEED * 0.3 * std::clamp(angle_error, -1.0, 1.0);  // невеликий корекційний поворот
        cmd_vel_pub_->publish(twist);

        RCLCPP_DEBUG(
            this->get_logger(),
            "Moving to marker (linear.x: %.3f, angular.z: %.3f)",
            twist.linear.x, twist.angular.z);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionMasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
