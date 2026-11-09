// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_err.h"


//static const char* TAG = "xyMotor";

namespace driver {

class xyMotorDriver {
    gpio_num_t enaPin;
    gpio_num_t dirPin;
    gpio_num_t pulsePin;

    static constexpr size_t MAX_STEPS = 500;
    static constexpr uint32_t MIN_HALF_US = 3;
    static constexpr uint32_t MAX_HALF_US = 32767;

    rmt_channel_handle_t channel;
    rmt_encoder_handle_t copyEnc;
    rmt_symbol_word_t symbols[MAX_STEPS]{};
    bool busy = false;

public:
    xyMotorDriver(gpio_num_t, gpio_num_t, gpio_num_t);
    void init();
    //non-blocking move function
    bool startSteps(uint32_t steps, uint32_t segmentUs);
    void waitDone();
    //blocking move function (turns one motor at a time)
    void moveSteps(size_t, uint32_t); //size_t because the rmt library defines that type of input that way
    esp_err_t onOff(uint8_t);  //when used must be in ESP_ERROR_CHECK()
    esp_err_t dir(uint8_t);


};
}