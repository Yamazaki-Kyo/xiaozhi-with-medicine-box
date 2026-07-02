#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

/**
 * 舵机控制 — LEDC PWM, 50Hz, 0.5~2.5ms → 0°~180°
 * 独立 Timer + Channel，不与电机共享
 */
class ServoController {
private:
    gpio_num_t pin_;
    ledc_channel_t channel_;
    int current_angle_;

    static constexpr int MIN_PULSE_US = 500;
    static constexpr int MAX_PULSE_US = 2500;

    int AngleToDuty(int angle);

public:
    ServoController();
    void Init(gpio_num_t pin, ledc_channel_t ch, ledc_timer_t timer);

    /// angle: 0~180度
    void SetAngle(int angle);
    int GetAngle() const;
};
