#include <rclcpp/rclcpp.hpp>


constexpr int ENA = 17;
constexpr int IN1 = 27;
constexpr int IN2 = 22;

constexpr int ENB = 13;
constexpr int IN3 = 26;
constexpr int IN4 = 19;

constexpr int PWM_FREQ = 700;

constexpr int LEFT_MIN_PWM = 90;
constexpr int RIGHT_MIN_PWM = 90;

constexpr float LEFT_GAIN = 1.00;
constexpr float RIGHT_GAIN = 1.00;
constexpr float LEFT_FORWARD_GAIN = 1.00;
constexpr float RIGHT_FOWRARD_GAIN = 1.00;
constexpr float LEFT_BACKWARD_GAIN = 1.00;
constexpr float RIGHT_BACKWARD_GAIN = 1.00;

constexpr bool LEFT_INVERT = false;
constexpr bool RIGHT_INVERT = false;

constexpr float WHEEL_BASE = 0.16;
constexpr float MAX_LINIAR = 0.2;
constexpr float MAX_ANGULAR = 2.0;

constexpr float CMD_TIMEOUT = 0.5;

class MotorNode : public rclcpp::Node
{
    public:
    MotorNode() : Node("motor_node"),

}
