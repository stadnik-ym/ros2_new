#include <algorithm>
#include <array>
#include <cstdlib>
#include <lgpio.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/nav_msgs/msg/odometry.hpp>
#include <stdexcept>
#include <vector>
#include "../include/diff_drive/artemka.hpp"
// #include <gpiod.hpp>
#include <gpiod.h>

class MotorNode : public rclcpp::Node
{
    public:
    MotorNode() : Node("motor_node")
    {
    RCLCPP_INFO(this->get_logger(), "Мотор нода була запущена");
    chip_ = lgGpiochipOpen(4);
    if (chip_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "не вдалось відкрити GPIO чіп");
        throw std::runtime_error("GPIO init failed");
    }
    setup_gpio();
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&MotorNode::cmd_cb, this, std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    };
    private:
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::nav_msgs::msg::Odometry>::SharedPtr publisher_;
    int chip_;

    void setup_gpio()
    {
        auto pins = std::array<int, 6>{artemka::ENA, artemka::IN1, artemka::IN2, artemka::ENB, artemka::IN3, artemka::IN4};
        for (auto p : pins)
        {
            lgGpioClaimOutput(this->chip_, 0, p, 0);
        }
        stop();
    }

    void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float v = std::clamp(static_cast<float>(msg->linear.x), -artemka::MAX_LINEAR, artemka::MAX_LINEAR);
        float w = std::clamp(static_cast<float>(msg->angular.z), -artemka::MAX_ANGULAR, artemka::MAX_ANGULAR);

        auto last_cmd = this->now();

        if (std::abs(v) < 0.001f && std::abs(w) < 0.001f)
        {
            stop();
        }
    }

    int pwm_from_speed(float s, int min_pwm)
    {
       float speed = std::abs(s);
       if (speed < 0.001f)
       {
           return 0;
       }
       speed = std::clamp(speed, 0.0f, 1.0f);
       float duty = min_pwm + speed * (100.0 - min_pwm);
       duty = std::clamp(duty, 0.0f, 100.0f);

       return static_cast<int>(duty);
    }

    float apply_left_comprassion(float s)
    {
        if (artemka::LEFT_INVERT)
        {
            s = -s;
        }

        s *= artemka::LEFT_GAIN;
        if (s < 0)
        {
            s *= artemka::LEFT_FORWARD_GAIN;
        } else if (s > 0)
        {
            s *= artemka::RIGHT_FORWARD_GAIN;
        }
        return std::clamp(s, -1.0f, 1.0f);
    }

    float apply_right_comprassion(float s)
    {
        if (artemka::RIGHT_INVERT)
        {
            s = -s;
        }

        s *= artemka::RIGHT_GAIN;
        if (s < 0)
        {
            s *= artemka::RIGHT_FORWARD_GAIN;
        } else if (s > 0)
        {
            s *= artemka::LEFT_FORWARD_GAIN;
        }
        return std::clamp(s, -1.0f, 1.0f);
    }

    void set_left(float s)
    {
        s = this->apply_left_comprassion(s);
        auto duty = this->pwm_from_speed(s, artemka::LEFT_MIN_PWM);

        if (s > 0)
        {
            lgGpioWrite(this->chip_, artemka::IN1, 1);
            lgGpioWrite(this->chip_, artemka::IN2, 0);
        } else if (s < 0)
        {
            lgGpioWrite(this->chip_, artemka::IN1, 0);
            lgGpioWrite(this->chip_, artemka::IN2, 1);
        } else
        {
            lgGpioWrite(this->chip_, artemka::IN1, 0);
            lgGpioWrite(this->chip_, artemka::IN2, 0);
        }
    }

    void set_right(float s)
    {
        s = this->apply_left_comprassion(s);
        auto duty = this->pwm_from_speed(s, artemka::LEFT_MIN_PWM);

        if (s > 0)
        {
            lgGpioWrite(this->chip_, artemka::IN3, 0);
            lgGpioWrite(this->chip_, artemka::IN4, 1);
        } else if (s < 0)
        {
            lgGpioWrite(this->chip_, artemka::IN3, 1);
            lgGpioWrite(this->chip_, artemka::IN4, 0);
        } else
        {
            lgGpioWrite(this->chip_, artemka::IN3, 0);
            lgGpioWrite(this->chip_, artemka::IN4, 0);
        }
    }

    void stop()
    {
        lgTxPwm(this->chip_, artemka::ENA,  artemka::PWM_FREQ, 0.0f, 0, 0);
        lgTxPwm(this->chip_, artemka::ENB, artemka::PWM_FREQ, 0.0f, 0, 0);

        auto direction_pins = std::array<int, 4> {artemka::IN1, artemka::IN2, artemka::IN3, artemka::IN4};

        for (auto p : direction_pins)
        {
            lgGpioWrite(this->chip_, p, 0);
        }
    }

    void update()
    {
        // now = now();

        auto dt = now - this->get_clock()->last_time();
    }

    void destroy_node()
    {
        lgGpiochipClose(this->chip_);
    }
};
