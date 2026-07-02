#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

/**
 * DRV8833 单路 H 桥电机驱动
 *
 * DRV8833 真值表:
 *   IN1=0   IN2=0   → Coast (滑行停止)
 *   IN1=PWM IN2=0   → 正转
 *   IN1=0   IN2=PWM → 反转
 *   IN1=1   IN2=1   → Brake (刹车)
 *
 * LEDC 10-bit PWM @ 5kHz 实现调速
 */
class MotorController {
private:
    gpio_num_t in1_pin_, in2_pin_;
    ledc_channel_t ch_in1_, ch_in2_;
    int current_duty_;      // 0~1023 (10-bit)
    int current_dir_;       // 1=正转, -1=反转, 0=停止

public:
    MotorController();
    void Init(gpio_num_t in1, gpio_num_t in2, ledc_channel_t ch1, ledc_channel_t ch2, ledc_timer_t timer);

    /// speed: 0~100（百分比）, dir: 1=正转, -1=反转
    void Run(int speed, int dir);
    void Stop();

    int GetSpeed() const;
    int GetDir() const;
};
