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

static const char* kTag = "Elatch";

static const uint32_t kStackSize = 4096;
static const uint32_t kTaskPollingInterval = 100;

Elatch::Elatch(bool has_sensor) : has_sensor_(has_sensor) {
    Lock();
    queue_ = xQueueCreate(10, sizeof(uint32_t));
    BaseType_t res = xTaskCreate(
        TaskForwarder, "trigger_task", kStackSize, this, uxTaskPriorityGet(nullptr), &task_handle_);

    assert(res == pdPASS);
}

void Elatch::Lock() { locked_ = true; }
void Elatch::Unlock(uint16_t pulse_duration_ms, uint16_t rest_duration_ms) {
    locked_ = false;
    ElatchMessage message = {pulse_duration_ms, rest_duration_ms};
    xQueueSend(queue_, &message, 0);
}

Elatch::~Elatch() { vTaskDelete(task_handle_); }

void Elatch::Pulse(uint16_t duration_ms) {
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
    ElatchMessage message;
    uint16_t pulse_duration_ms = 0;
    uint16_t rest_duration_ms = 0;

    while (1) {
        BaseType_t res = xQueueReceive(queue_, &message, pdMS_TO_TICKS(kTaskPollingInterval));
        if (res != pdPASS && locked_) {
            continue;
        }
        if (res == pdPASS) {
            pulse_duration_ms = message.pulse_duration_ms;
            rest_duration_ms = message.rest_duration_ms;
        }
        if (pulse_duration_ms <= 0) {
            ESP_LOGW(kTag, "Invalid pulse duration");
        }

        Pulse(pulse_duration_ms);
        vTaskDelay(pdMS_TO_TICKS(rest_duration_ms));
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
