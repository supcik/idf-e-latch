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
static const int queue_size = 10;

Elatch::Elatch(bool has_sensor, int32_t pulse_duration_ms, int32_t rest_duration_ms)
    : has_sensor_(has_sensor),
      pulse_duration_ms_(pulse_duration_ms),
      rest_duration_ms_(rest_duration_ms) {
    queue_ = xQueueCreate(queue_size, sizeof(ElatchMessage));
    BaseType_t res = xTaskCreate(
        TaskForwarder, "trigger_task", kStackSize, this, uxTaskPriorityGet(nullptr), &task_handle_);
    assert(res == pdPASS);
    Lock();
}

void Elatch::Lock() {
    ElatchMessage message = {want_to_open : false};
    xQueueSend(queue_, &message, portMAX_DELAY);
}

void Elatch::Unlock() {
    ElatchMessage message = {want_to_open : true};
    xQueueSend(queue_, &message, portMAX_DELAY);
}

Elatch::~Elatch() { vTaskDelete(task_handle_); }

void Elatch::Pulse(uint16_t duration_ms) {
    Pull();
    int64_t t0 = esp_timer_get_time() / 1000;
    while (1) {
        if (has_sensor_ && !isClosed()) {
            break;
        }
        int64_t t1 = esp_timer_get_time() / 1000;
        if (t1 - t0 > duration_ms) {
            break;
        }
        vTaskDelay(1);
    }

    Release();
}

void Elatch::Task() {
    ElatchMessage message;

    int64_t last_pulse = 0;
    bool need_pulse = false;
    bool want_to_open = false;

    while (1) {
        BaseType_t res = xQueueReceive(queue_, &message, pdMS_TO_TICKS(kTaskPollingInterval));

        if (res == pdPASS) {
            want_to_open = message.want_to_open;
            need_pulse = want_to_open;
        }

        if (has_sensor_ && want_to_open) {
            if (isClosed()) {
                need_pulse = true;
            } else {  // already open, skip pulse
                need_pulse = false;
            }
        }

        int64_t now = esp_timer_get_time() / 1000;

        if (need_pulse && (now - last_pulse > rest_duration_ms_)) {
            ESP_LOGI(kTag,
                     "Pulse %" PRId32 " ms, rest %" PRId32 " ms",
                     pulse_duration_ms_,
                     rest_duration_ms_);
            Pulse(pulse_duration_ms_);
            need_pulse = false;
            last_pulse = now;
        }
    }
}

GpioElatch::GpioElatch(gpio_num_t actuator_pin,
                       gpio_num_t sensor_pin,
                       bool inverted_actuator_logic,
                       bool inverted_sensor_logic,
                       int32_t pulse_duration_ms,
                       int32_t rest_duration_ms)
    : Elatch(sensor_pin >= 0, pulse_duration_ms, rest_duration_ms),
      actuator_pin_(actuator_pin),
      sensor_pin_(sensor_pin),
      inverted_actuator_logic_(inverted_actuator_logic),
      inverted_sensor_logic_(inverted_sensor_logic) {
    Release();
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

void GpioElatch::Pull() { gpio_set_level(actuator_pin_, inverted_actuator_logic_ ? 0 : 1); }
void GpioElatch::Release() { gpio_set_level(actuator_pin_, inverted_actuator_logic_ ? 1 : 0); }
bool GpioElatch::isClosed() {
    return gpio_get_level(sensor_pin_) == inverted_sensor_logic_ ? 1 : 0;
}
