/**
 ******************************************************************************
 * @file        : e_latch.cpp
 * @brief       : Electric Latch driver
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 11 March 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include "e_latch.hpp"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const uint32_t kStackSize = 4096;
static const uint32_t kPulseTime = 100;
static const uint32_t kRestTime = 1000;

Elatch::Elatch(bool has_sensor) : has_sensor_(has_sensor) {
    Lock();
    queue_ = xQueueCreate(10, sizeof(uint32_t));
    BaseType_t res = xTaskCreate(
        TaskForwarder, "trigger_task", kStackSize, this, uxTaskPriorityGet(nullptr), &task_handle_);

    assert(res == pdPASS);
}

void Elatch::Lock() { locked_ = true; }
void Elatch::Unlock() {
    locked_ = false;
    uint32_t message = 0;
    xQueueSend(queue_, &message, 0);
}

Elatch::~Elatch() { vTaskDelete(task_handle_); }

void Elatch::Pulse(uint32_t duration_ms) {
    Pull();
    int64_t t0 = esp_timer_get_time();
    while (1) {
        if (!isClosed()) {
            break;
        }
        int64_t t1 = esp_timer_get_time();
        if (t1 - t0 > duration_ms * 1000) {
            break;
        }
        vTaskDelay(1);
    }

    Release();
}

void Elatch::Task() {
    uint32_t message;
    while (1) {
        BaseType_t res = xQueueReceive(queue_, &message, pdMS_TO_TICKS(100));
        if (res == pdPASS || !locked_) {
            Pulse(kPulseTime);
            vTaskDelay(pdMS_TO_TICKS(kRestTime));
        }
    }
}

GpioElatch::GpioElatch(gpio_num_t actuator_pin, gpio_num_t sensor_pin)
    : Elatch(sensor_pin >= 0), actuator_pin_(actuator_pin), sensor_pin_(sensor_pin) {
    gpio_set_level(actuator_pin_, 0);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << actuator_pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (sensor_pin_ >= 0) {
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << sensor_pin_);
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }
}

void GpioElatch::Pull() { gpio_set_level(actuator_pin_, 1); }
void GpioElatch::Release() { gpio_set_level(actuator_pin_, 0); }
bool GpioElatch::isClosed() { return gpio_get_level(sensor_pin_) == 0; }
