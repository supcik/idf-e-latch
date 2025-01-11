/**
 ******************************************************************************
 * @file        : main.cpp
 * @brief       : Electric Latch example
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 20 November 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include <inttypes.h>
#include <unistd.h>

#include "e_latch.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "status_led.hpp"

extern "C" {
void app_main(void);
}

static const char* kTag = "app";

static const gpio_num_t kActuatorPin = GPIO_NUM_14;
static const gpio_num_t kSensorPin = GPIO_NUM_21;

void app_main(void) {
    ESP_LOGI(kTag, "Starting App");

    status_led::LedDevice* led_device = new status_led::Ws2812Led(47, true);
    StatusLed* led = new StatusLed(led_device);

    GpioElatch latch(kActuatorPin, kSensorPin);

    while (true) {
        ESP_LOGI(kTag, "Locking");
        led->On(kRed);
        latch.Lock();
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(kTag, "Unlocking");
        led->On(kGreen);
        latch.Unlock();
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
