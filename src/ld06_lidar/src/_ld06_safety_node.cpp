#include <chrono>
#include <memory>
#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class LidarSafety : public rclcpp::Node
{
public:
    LidarSafety()
        : Node("lidar_safety")
    {
        declare_parameter("input_cmd_topic", "/cmd_vel_raw");
        declare_parameter("output_cmd_topic", "/cmd_vel");
        declare_parameter("front_distance_topic", "/lidar/front_distance");
        declare_parameter("safety_stop_topic", "/safety/lidar_stop");

        declare_parameter("cmd_timeout_sec", 0.5);
        declare_parameter("lidar_timeout_sec", 0.7);

        declare_parameter("stop_distance_m", 0.22);
        declare_parameter("clear_distance_m", 0.30);

        declare_parameter("invalid_front_timeout_sec", 1.0);

        declare_parameter("allow_rotation_when_blocked", true);
        declare_parameter("allow_reverse_when_blocked", true);

        declare_parameter("max_linear_x", 0.35);
        declare_parameter("max_angular_z", 1.5);

        declare_parameter("enable_slowdown", true);
        declare_parameter("slowdown_distance_m", 0.80);
        declare_parameter("min_slowdown_factor", 0.25);

        get_parameter("input_cmd_topic", input_cmd_topic_);
        get_parameter("output_cmd_topic", output_cmd_topic_);
        get_parameter("front_distance_topic", front_distance_topic_);
        get_parameter("safety_stop_topic", safety_stop_topic_);

        get_parameter("cmd_timeout_sec", cmd_timeout_sec_);
        get_parameter("lidar_timeout_sec", lidar_timeout_sec_);

        get_parameter("stop_distance_m", stop_distance_m_);
        get_parameter("clear_distance_m", clear_distance_m_);

        get_parameter(
            "invalid_front_timeout_sec",
            invalid_front_timeout_sec_);

        get_parameter(
            "allow_rotation_when_blocked",
            allow_rotation_when_blocked_);

        get_parameter(
            "allow_reverse_when_blocked",
            allow_reverse_when_blocked_);

        get_parameter("max_linear_x", max_linear_x_);
        get_parameter("max_angular_z", max_angular_z_);

        get_parameter("enable_slowdown", enable_slowdown_);
        get_parameter("slowdown_distance_m", slowdown_distance_m_);
        get_parameter("min_slowdown_factor", min_slowdown_factor_);

        max_linear_x_ = std::abs(max_linear_x_);
        max_angular_z_ = std::abs(max_angular_z_);

        min_slowdown_factor_ =
            std::clamp(min_slowdown_factor_, 0.0, 1.0);

        last_cmd_time_ = now_sec();
        last_lidar_time_ = now_sec();
        last_valid_front_time_ = 0.0;

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            input_cmd_topic_,
            1,
            std::bind(
                &LidarSafety::cmdCallback,
                this,
                std::placeholders::_1));

        front_sub_ = create_subscription<std_msgs::msg::Float32>(
            front_distance_topic_,
            1,
            std::bind(
                &LidarSafety::frontDistanceCallback,
                this,
                std::placeholders::_1));

        cmd_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                output_cmd_topic_,
                1);

        stop_pub_ =
            create_publisher<std_msgs::msg::Bool>(
                safety_stop_topic_,
                10);

        timer_ = create_wall_timer(
            10ms,
            std::bind(
                &LidarSafety::controlLoop,
                this));

        RCLCPP_INFO(
            get_logger(),
            "Lidar safety started: %s -> %s, front=%s stop=%.2f clear=%.2f",
            input_cmd_topic_.c_str(),
            output_cmd_topic_.c_str(),
            front_distance_topic_.c_str(),
            stop_distance_m_,
            clear_distance_m_);
    }

private:
    double now_sec() const
    {
        return this->get_clock()->now().seconds();
    }

    void cmdCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_ = *msg;
        last_cmd_time_ = now_sec();
    }

    void frontDistanceCallback(
        const std_msgs::msg::Float32::SharedPtr msg)
    {
        double now = now_sec();

        float distance = msg->data;

        last_lidar_time_ = now;

        if (distance >= 0.0f)
        {
            front_distance_ = distance;
            last_valid_front_time_ = now;

            if (distance <= stop_distance_m_)
            {
                front_blocked_ = true;

                if (last_cmd_.linear.x > 0.0)
                {
                    auto stop_cmd = makeStopCmd();

                    if (allow_rotation_when_blocked_)
                    {
                        stop_cmd.angular.z =
                            last_cmd_.angular.z;
                    }

                    cmd_pub_->publish(stop_cmd);
                    publishStopState(true);
                }
            }
        }
    }

    geometry_msgs::msg::Twist makeStopCmd()
    {
        return geometry_msgs::msg::Twist();
    }

    geometry_msgs::msg::Twist clampCmd(
        geometry_msgs::msg::Twist cmd)
    {
        cmd.linear.x =
            std::clamp(
                cmd.linear.x,
                -max_linear_x_,
                max_linear_x_);

        cmd.angular.z =
            std::clamp(
                cmd.angular.z,
                -max_angular_z_,
                max_angular_z_);

        cmd.linear.y = 0.0;

        return cmd;
    }

    bool validFrontDistance()
    {
        double now = now_sec();

        return
            front_distance_ >= 0.0 &&
            (now - last_valid_front_time_)
                <= invalid_front_timeout_sec_;
    }

    void updateBlockedState()
    {
        if (!validFrontDistance())
        {
            return;
        }

        if (front_blocked_)
        {
            if (front_distance_ >= clear_distance_m_)
            {
                front_blocked_ = false;
            }
        }
        else
        {
            if (front_distance_ <= stop_distance_m_)
            {
                front_blocked_ = true;
            }
        }
    }

    double slowdownFactor()
    {
        if (!enable_slowdown_)
            return 1.0;

        if (!validFrontDistance())
            return 1.0;

        if (front_distance_ >= slowdown_distance_m_)
            return 1.0;

        if (slowdown_distance_m_ <= 0.01)
            return 1.0;

        double factor =
            front_distance_ / slowdown_distance_m_;

        return std::clamp(
            factor,
            min_slowdown_factor_,
            1.0);
    }

    void publishStopState(bool active)
    {
        std_msgs::msg::Bool msg;
        msg.data = active;
        stop_pub_->publish(msg);
    }

    void controlLoop()
    {
        double now = now_sec();

        if ((now - last_cmd_time_) > cmd_timeout_sec_)
        {
            cmd_pub_->publish(makeStopCmd());
            publishStopState(true);
            return;
        }

        if ((now - last_lidar_time_) > lidar_timeout_sec_)
        {
            RCLCPP_WARN(
                get_logger(),
                "LIDAR TIMEOUT: stopping robot");

            cmd_pub_->publish(makeStopCmd());
            publishStopState(true);
            return;
        }

        updateBlockedState();

        auto cmd = last_cmd_;

        cmd = clampCmd(cmd);

        bool moving_forward = cmd.linear.x > 0.0;
        bool moving_backward = cmd.linear.x < 0.0;

        bool safety_active = false;
        std::string reason = "OK";

        if (front_blocked_ && moving_forward)
        {
            cmd.linear.x = 0.0;

            safety_active = true;
            reason = "front blocked";

            if (!allow_rotation_when_blocked_)
            {
                cmd.angular.z = 0.0;
            }
        }

        if (front_blocked_ &&
            moving_backward &&
            !allow_reverse_when_blocked_)
        {
            cmd.linear.x = 0.0;

            safety_active = true;
            reason =
                "reverse disabled while blocked";
        }

        if (!front_blocked_ && moving_forward)
        {
            double factor = slowdownFactor();

            if (factor < 1.0)
            {
                cmd.linear.x *= factor;

                safety_active = true;

                reason =
                    "slowdown " +
                    std::to_string(factor);
            }
        }

        cmd = clampCmd(cmd);

        cmd_pub_->publish(cmd);
        publishStopState(safety_active);

        logStatus(
            cmd,
            safety_active,
            reason);
    }

    void logStatus(
        const geometry_msgs::msg::Twist &cmd,
        bool safety_active,
        const std::string &reason)
    {
        double now = now_sec();

        if ((now - last_log_) < 0.5)
            return;

        last_log_ = now;

        if (safety_active)
        {
            RCLCPP_WARN(
                get_logger(),
                "SAFETY: %s | front=%.3f m | out linear.x=%.3f angular.z=%.3f",
                reason.c_str(),
                front_distance_,
                cmd.linear.x,
                cmd.angular.z);
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "OK | front=%.3f m | out linear.x=%.3f angular.z=%.3f",
                front_distance_,
                cmd.linear.x,
                cmd.angular.z);
        }
    }

private:
    std::string input_cmd_topic_;
    std::string output_cmd_topic_;
    std::string front_distance_topic_;
    std::string safety_stop_topic_;

    double cmd_timeout_sec_;
    double lidar_timeout_sec_;

    double stop_distance_m_;
    double clear_distance_m_;

    double invalid_front_timeout_sec_;

    bool allow_rotation_when_blocked_;
    bool allow_reverse_when_blocked_;

    double max_linear_x_;
    double max_angular_z_;

    bool enable_slowdown_;
    double slowdown_distance_m_;
    double min_slowdown_factor_;

    geometry_msgs::msg::Twist last_cmd_;

    double last_cmd_time_{0.0};
    double last_lidar_time_{0.0};
    double last_valid_front_time_{0.0};

    double last_log_{0.0};

    double front_distance_{-1.0};

    bool front_blocked_{false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarSafety>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
