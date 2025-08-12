/**
 ******************************************************************************
 * @file        : e_latch.hpp
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

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class Elatch {
   public:
    Elatch(bool has_sensor, int32_t pulse_duration_ms = 500, int32_t rest_duration_ms = 5000);
    virtual ~Elatch();

    void Lock();
    void Unlock();
    virtual bool isClosed() { return false; };

   protected:
    virtual void Pull() = 0;
    virtual void Release() = 0;

   private:
    struct ElatchMessage {
        bool want_to_open;
    };

    bool has_sensor_;
    int32_t pulse_duration_ms_;
    int32_t rest_duration_ms_;

    QueueHandle_t queue_;
    TaskHandle_t task_handle_;

    void Pulse(uint16_t duration_ms);
    void Task();
    static void TaskForwarder(void* param) {
        Elatch* latch = (Elatch*)param;
        latch->Task();
    }
};

class GpioElatch : public Elatch {
   public:
    GpioElatch(gpio_num_t actuator_pin,
               gpio_num_t sensor_pin,
               bool inverted_actuator_logic = false,
               bool inverted_sensor_logic = false,
               int32_t pulse_duration_ms = 500,
               int32_t rest_duration_ms = 5000);
    bool isClosed() override;

   protected:
    void Pull() override;
    void Release() override;

   private:
    gpio_num_t actuator_pin_;
    gpio_num_t sensor_pin_;
    bool inverted_actuator_logic_;
    bool inverted_sensor_logic_;
};
