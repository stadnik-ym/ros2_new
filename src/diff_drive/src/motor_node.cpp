#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <array>

#include <lgpio.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/nav_msgs/msg/odometry.hpp>

// Предполагаем, что пины лежат здесь, либо замени на свои константы
// #include "../include/diff_drive/artemka.hpp"

namespace artemka {
    const int ENA = 17;
    const int IN1 = 27;
    const int IN2 = 22;
    const int ENB = 13;
    const int IN3 = 26;
    const int IN4 = 19;
}

class DiffDriveNode : public rclcpp::Node {
public:
    DiffDriveNode() : Node("diff_drive_node") {
        RCLCPP_INFO(this->get_logger(), "Starting DiffDrive with motor compensation (C++)");

        chip_ = lgGpiochipOpen(4);
        if (chip_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip 4. Error code: %d", chip_);
            throw std::runtime_error("GPIO init failed");
        }
        setup_gpio();

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&DiffDriveNode::cmd_cb, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        last_time_ = this->get_clock()->now();
        last_cmd_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&DiffDriveNode::update, this));

        RCLCPP_INFO(this->get_logger(), "DiffDrive Ready. Listening /cmd_vel");
    }

    ~DiffDriveNode() {
        shutdown_hardware();
    }

private:
    // Константы управления
    const int PWM_FREQ = 700;
    const int LEFT_MIN_PWM = 90;
    const int RIGHT_MIN_PWM = 90;

    const double LEFT_GAIN = 1.00;
    const double RIGHT_GAIN = 1.00;
    const double LEFT_FORWARD_GAIN = 1.00;
    const double RIGHT_FORWARD_GAIN = 1.00;
    const double LEFT_BACKWARD_GAIN = 1.00;
    const double RIGHT_BACKWARD_GAIN = 1.00;

    const bool LEFT_INVERT = false;
    const bool RIGHT_INVERT = false;

    const double WHEEL_BASE = 0.16;
    const double MAX_LINEAR = 0.2;
    const double MAX_ANGULAR = 2.0;
    const double CMD_TIMEOUT = 0.5; // в секундах

    // змінні стану
    int chip_ = -1;
    double v_ = 0.0;
    double w_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setup_gpio() {
        std::array<int, 6> pins = {artemka::ENA, artemka::IN1, artemka::IN2,
                                   artemka::ENB, artemka::IN3, artemka::IN4};
        for (int p : pins) {
            lgGpioClaimOutput(chip_, 0, p, 0);
        }
        stop();
    }

    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
        v_ = std::clamp(msg->linear.x, -MAX_LINEAR, MAX_LINEAR);
        w_ = std::clamp(msg->angular.z, -MAX_ANGULAR, MAX_ANGULAR);
        last_cmd_time_ = this->get_clock()->now();

        if (std::abs(v_) < 0.001 && std::abs(w_) < 0.001) {
            stop();
        }
    }

    double pwm_from_speed(double s, int min_pwm) {
        double speed = std::abs(s);
        if (speed < 0.001) return 0.0;

        speed = std::clamp(speed, 0.0, 1.0);

        double duty = min_pwm + speed * (100.0 - min_pwm);
        return std::clamp(duty, 0.0, 100.0);
    }

    double apply_left_compensation(double s) {
        if (LEFT_INVERT) s = -s;
        s *= LEFT_GAIN;

        if (s > 0)       s *= LEFT_FORWARD_GAIN;
        else if (s < 0)  s *= LEFT_BACKWARD_GAIN;

        return std::clamp(s, -1.0, 1.0);
    }

    double apply_right_compensation(double s) {
        if (RIGHT_INVERT) s = -s;
        s *= RIGHT_GAIN;

        if (s > 0)       s *= RIGHT_FORWARD_GAIN;
        else if (s < 0)  s *= RIGHT_BACKWARD_GAIN;

        return std::clamp(s, -1.0, 1.0);
    }

    void set_left(double s) {
        s = apply_left_compensation(s);
        int duty = pwm_from_speed(s, LEFT_MIN_PWM);

        if (s > 0) {
            lgGpioWrite(chip_, artemka::IN1, 1);
            lgGpioWrite(chip_, artemka::IN2, 0);
        } else if (s < 0) {
            lgGpioWrite(chip_, artemka::IN1, 0);
            lgGpioWrite(chip_, artemka::IN2, 1);
        } else {
            lgGpioWrite(chip_, artemka::IN1, 0);
            lgGpioWrite(chip_, artemka::IN2, 0);
        }
        // Сишная функция: lgTxPwm(handle, pin, freq, duty_cycle_0_to_1000000)
        lgTxPwm(chip_, artemka::ENA, PWM_FREQ, duty, 0, 0);
    }

    void set_right(double s) {
        s = apply_right_compensation(s);
        int duty = pwm_from_speed(s, RIGHT_MIN_PWM);

        if (s > 0) {
            lgGpioWrite(chip_, artemka::IN3, 1);
            lgGpioWrite(chip_, artemka::IN4, 0);
        } else if (s < 0) {
            lgGpioWrite(chip_, artemka::IN3, 0);
            lgGpioWrite(chip_, artemka::IN4, 1);
        } else {
            lgGpioWrite(chip_, artemka::IN3, 0);
            lgGpioWrite(chip_, artemka::IN4, 0);
        }
        lgTxPwm(chip_, artemka::ENB, PWM_FREQ, duty, 0, 0);
    }

    void stop() {
        lgTxPwm(chip_, artemka::ENA, PWM_FREQ, 0.0f, 0, 0);
        lgTxPwm(chip_, artemka::ENB, PWM_FREQ, 0.0f, 0, 0);
        lgGpioWrite(chip_, artemka::IN1, 0);
        lgGpioWrite(chip_, artemka::IN2, 0);
        lgGpioWrite(chip_, artemka::IN3, 0);
        lgGpioWrite(chip_, artemka::IN4, 0);
    }

    void update() {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if ((now - last_cmd_time_).seconds() > CMD_TIMEOUT) {
            v_ = 0.0;
            w_ = 0.0;
        }

        double vl = v_ - w_ * WHEEL_BASE / 2.0;
        double vr = v_ + w_ * WHEEL_BASE / 2.0;

        double maxv = std::max(std::abs(vl), std::abs(vr));
        if (maxv > MAX_LINEAR) {
            double k = MAX_LINEAR / maxv;
            vl *= k;
            vr *= k;
        }

        double left_input = vl / MAX_LINEAR;
        double right_input = vr / MAX_LINEAR;

        set_left(left_input);
        set_right(right_input);

        double actual_v = (vl + vr) / 2.0;
        double actual_w = (vr - vl) / WHEEL_BASE;

        theta_ += actual_w * dt;
        x_ += actual_v * std::cos(theta_) * dt;
        y_ += actual_v * std::sin(theta_) * dt;

        publish_odom(actual_v, actual_w);
    }

    void publish_odom(double v, double w) {
        auto msg = nav_msgs::msg::Odometry();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = y_;

        msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

        msg.twist.twist.linear.x = v;
        msg.twist.twist.angular.z = w;

        odom_pub_->publish(msg);
    }

    void shutdown_hardware() {
        if (chip_ >= 0) {
            stop();
            lgGpiochipClose(chip_);
            chip_ = -1;
            RCLCPP_INFO(this->get_logger(), "GPIO Chip closed successfully.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
