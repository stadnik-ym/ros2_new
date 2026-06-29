// #include "std_msgs/msg/bool.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <gpiod.hpp>
// #include <std_msgs/std_msgs/msg/bool.hpp>

// class BuzzerNode : public rclcpp::Node
// {
//     public:
//     BuzzerNode() : Node("buzzer_node")
//     {
//         chip.open("gpiochip4");
//         line = chip.get_line(18);
//         line.req
//     }
//     private:
//     void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
//     {
//         line.set_value(msg->data ? 1 : 0);
//         RCLCPP_INFO(this->get_logger(), "Buzzer state: '%s'", msg->data ? "ON" : "OFF");
//     }

//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscruption_;
//     gpiod::chip chip;
//     gpiod::line line;
// };

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
// В зависимости от платформы, используйте нужную библиотеку GPIO
// Для Raspberry Pi часто используют gpiod или bcm2835
#include <gpiod.hpp>

class BuzzerNode : public rclcpp::Node {
public:
    BuzzerNode() : Node("buzzer_node") {
        // 1. Открываем чип (например, gpiochip4 - проверьте через команду `gpioinfo`)
        gpiod::chip chip("/dev/gpiochip4");

        // 2. Настраиваем запрос на вывод
        gpiod::line_config lcfg;
        lcfg.add_line_settings(18, gpiod::line_settings().set_direction(gpiod::line::direction::OUTPUT));

        gpiod::request_config rcfg;
        rcfg.set_consumer("buzzer_node");

        // 3. Выполняем запрос
        request = chip.request_lines(rcfg, lcfg);

        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/buzzer/command", 10,
            std::bind(&BuzzerNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // 4. Устанавливаем значение (через словарь значений по смещению пина)
        request.set_value(18, msg->data ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);
        RCLCPP_INFO(this->get_logger(), "Buzzer state: '%s'", msg->data ? "ON" : "OFF");
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    gpiod::line_request request; // Теперь используем line_request
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BuzzerNode>());
    rclcpp::shutdown();
    return 0;
}
