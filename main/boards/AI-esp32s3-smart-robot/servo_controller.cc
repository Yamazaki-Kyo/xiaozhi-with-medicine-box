#include "servo_controller.h"
#include <esp_log.h>

#define TAG "Servo"

int ServoController::AngleToDuty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    float pulse_us = MIN_PULSE_US + (float)(MAX_PULSE_US - MIN_PULSE_US) * angle / 180.0f;
    // 14-bit timer @ 50Hz → max_duty=16384, 每μs=16384/20000≈0.8192
    return (int)(pulse_us * 16384.0f / 20000.0f);
}

ServoController::ServoController()
    : pin_(GPIO_NUM_NC), channel_(LEDC_CHANNEL_MAX), current_angle_(90) {}

void ServoController::Init(gpio_num_t pin, ledc_channel_t ch, ledc_timer_t timer) {
    pin_ = pin;
    channel_ = ch;
    if (pin_ == GPIO_NUM_NC) return;

    ledc_channel_config_t cfg = {};
    cfg.gpio_num = pin_;
    cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg.channel = channel_;
    cfg.timer_sel = timer;
    cfg.duty = AngleToDuty(90);
    cfg.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
    current_angle_ = 90;
}

void ServoController::SetAngle(int angle) {
    if (pin_ == GPIO_NUM_NC) return;
    int duty = AngleToDuty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_));
    current_angle_ = angle;
    ESP_LOGI(TAG, "GPIO%d → %d°", pin_, angle);
}

int ServoController::GetAngle() const { return current_angle_; }
