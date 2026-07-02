#include "motor_controller.h"
#include <esp_log.h>

#define TAG "Motor"

MotorController::MotorController()
    : in1_pin_(GPIO_NUM_NC), in2_pin_(GPIO_NUM_NC),
      ch_in1_(LEDC_CHANNEL_MAX), ch_in2_(LEDC_CHANNEL_MAX),
      current_duty_(0), current_dir_(0) {}

void MotorController::Init(gpio_num_t in1, gpio_num_t in2, ledc_channel_t ch1, ledc_channel_t ch2, ledc_timer_t timer) {
    in1_pin_ = in1; in2_pin_ = in2;
    ch_in1_ = ch1; ch_in2_ = ch2;

    ledc_channel_config_t cfg1 = {};
    cfg1.gpio_num = in1_pin_;
    cfg1.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg1.channel = ch_in1_;
    cfg1.timer_sel = timer;
    cfg1.duty = 0;
    cfg1.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg1));

    ledc_channel_config_t cfg2 = {};
    cfg2.gpio_num = in2_pin_;
    cfg2.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg2.channel = ch_in2_;
    cfg2.timer_sel = timer;
    cfg2.duty = 0;
    cfg2.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg2));
}

void MotorController::Run(int speed, int dir) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    int duty = (speed * 1023) / 100;
    current_duty_ = duty;
    current_dir_ = dir;

    ESP_LOGI(TAG, "GPIO%d/%d dir=%d speed=%d duty=%d ch=%d/%d",
             in1_pin_, in2_pin_, dir, speed, duty, ch_in1_, ch_in2_);

    if (dir == 1) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1_, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1_));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2_, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2_));
    } else if (dir == -1) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1_, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1_));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2_, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2_));
    } else {
        Stop();
    }
}

void MotorController::Stop() {
    current_duty_ = 0;
    current_dir_ = 0;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in1_, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in1_));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_in2_, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_in2_));
}

int MotorController::GetSpeed() const { return (current_duty_ * 100) / 1023; }
int MotorController::GetDir() const { return current_dir_; }
