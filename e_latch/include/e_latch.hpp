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
    Elatch(bool has_sensor);
    virtual ~Elatch();

    void Lock();
    void Unlock(uint16_t pulse_duration_ms = 100, uint16_t rest_duration_ms = 1000);

   protected:
    virtual void Pull() = 0;
    virtual void Release() = 0;
    virtual bool isClosed() { return false; };

   private:
    struct ElatchMessage {
        uint16_t pulse_duration_ms;
        uint16_t rest_duration_ms;
    };

    bool has_sensor_;
    bool locked_ = true;

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
    GpioElatch(gpio_num_t actuator_pin, gpio_num_t sensor_pin);

   protected:
    void Pull() override;
    void Release() override;
    bool isClosed() override;

   private:
    gpio_num_t actuator_pin_;
    gpio_num_t sensor_pin_;
};
