// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include "joyStickDriver.hpp"

#include "joyStickDriver.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"




static const char* TAG = "DRIVER_J";

namespace driver {

JoyStickDriver::JoyStickDriver()
    : xRaw(0), yRaw(0), bPin(GPIO_NUM_19) {

}

void JoyStickDriver::init(){
    ESP_LOGI(TAG, "Initializing JoyStick driver");


    ESP_LOGI(TAG, "Configuing adc_oneshot");
    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = ADC_UNIT_1;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &joyStick)); //this make joyStick != nullptr

    ESP_LOGI(TAG, "Configuring adc_channel");
    //start configuring channel
    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = ADC_ATTEN_DB_12;
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    //configure x, y channels,
    ESP_ERROR_CHECK(adc_oneshot_config_channel(joyStick, xJoy, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(joyStick, yJoy, &chan_config));

    ESP_LOGI(TAG, "Configuring joystick button");
    gpio_config_t pinConfig = {};
    pinConfig.mode = GPIO_MODE_INPUT;
    pinConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pinConfig.pin_bit_mask = 1ULL << bPin;
    ESP_ERROR_CHECK(gpio_config(&pinConfig));

    ESP_LOGI(TAG, "Joystick Initialization complete");
}

bool JoyStickDriver::isPushed() {
    return gpio_get_level(bPin) == 0;
}

static int readMedian(adc_oneshot_unit_handle_t unit, adc_channel_t channel) {
    int input[7];
    for (int i = 0; i < 7; i++) ESP_ERROR_CHECK(adc_oneshot_read(unit, channel, &input[i]));
    for (int i = 1; i < 7; i++) {            // insertion sort
        int key = input[i];
        int j = i - 1;
        while (j >= 0 && input[j] > key) {
            input[j + 1] = input[j];
            j--;
        }
        input[j + 1] = key;
    }
    return input[3];
}

int JoyStickDriver::getRawX() {
    xRaw = readMedian(joyStick, xJoy);
    return xRaw;
}
int JoyStickDriver::getRawY() {
    yRaw = readMedian(joyStick, yJoy);
    return yRaw;
}

}
