#pragma once

namespace artemka {

// Піни драйверів моторів
constexpr int ENA = 17;
constexpr int IN1 = 27;
constexpr int IN2 = 22;

constexpr int ENB = 13;
constexpr int IN3 = 26;
constexpr int IN4 = 19;

// Параметри ШИМ
constexpr int PWM_FREQ = 700;
constexpr int LEFT_MIN_PWM = 90;
constexpr int RIGHT_MIN_PWM = 90;

// Коефіцієнти для калібрування
constexpr float LEFT_GAIN = 1.00f;
constexpr float RIGHT_GAIN = 1.00f;
constexpr float LEFT_FORWARD_GAIN = 1.00f;
constexpr float RIGHT_FORWARD_GAIN = 1.00f;
constexpr float LEFT_BACKWARD_GAIN = 1.00f;
constexpr float RIGHT_BACKWARD_GAIN = 1.00f;

// Направление вращения
constexpr bool LEFT_INVERT = false;
constexpr bool RIGHT_INVERT = false;

// Ліміти і кінематика
constexpr float WHEEL_BASE = 0.16f;
constexpr float MAX_LINEAR = 0.2f;
constexpr float MAX_ANGULAR = 2.0f;

// Безпека
constexpr float CMD_TIMEOUT = 0.5f;

}
