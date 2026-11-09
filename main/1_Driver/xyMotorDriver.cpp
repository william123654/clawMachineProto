// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include "xyMotorDriver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"

static const char* TAG = "MOTOR";

namespace driver {

xyMotorDriver::xyMotorDriver(gpio_num_t enaPin, gpio_num_t dirPin, gpio_num_t pulsePin) {
    this->enaPin = enaPin;
    this->dirPin = dirPin;
    this->pulsePin = pulsePin;
    this->channel = nullptr;
    this->copyEnc = nullptr;

    ESP_LOGI(TAG, "RMT Initialization");
    //1. create a channel bound to a GPIO and a timing resolution
    rmt_tx_channel_config_t channelConfig = {};
    channelConfig.gpio_num = pulsePin;
    channelConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    channelConfig.resolution_hz = 1000000; //1 Mhz = 1 micro-second / tick
    channelConfig.mem_block_symbols = 64; //hardware memory block, must be even 48-64;
    channelConfig.trans_queue_depth = 4; //transmit queue size

    ESP_ERROR_CHECK(rmt_new_tx_channel(&channelConfig, &channel));

    //2. create an encoder
    rmt_copy_encoder_config_t encConfig = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encConfig, &copyEnc));
    ESP_ERROR_CHECK(rmt_enable(channel));

    ESP_LOGI(TAG, "RMT Encoder Initialization complete");
}

void xyMotorDriver::init() {

    ESP_LOGI(TAG, "GPIO Initialization");
    gpio_config_t gConfig = {};
    gConfig.mode = GPIO_MODE_OUTPUT;;
    gConfig.pin_bit_mask = 1ULL<<enaPin | 1ULL<<dirPin;

    ESP_ERROR_CHECK(gpio_config(&gConfig));
}

    //passes in a time instead of a freq so that it performs x steps in y time instead of
    //x steps at y freq to make sure both motors start and stop at the same time. 
bool xyMotorDriver::startSteps(uint32_t steps, uint32_t segmentUs) {
    if (steps == 0 || steps > MAX_STEPS) return false;
    if (busy) waitDone();  // symbols[] is reused, so the previous burst must be finished

    uint32_t half = segmentUs / (2 * steps);  // each step = high half + low half
    if (half < MIN_HALF_US) half = MIN_HALF_US;
    if (half > MAX_HALF_US) half = MAX_HALF_US;

    for (uint32_t i = 0; i < steps; i++) {
        symbols[i].duration0 = static_cast<uint16_t>(half);
        symbols[i].level0 = 1;
        symbols[i].duration1 = static_cast<uint16_t>(half);
        symbols[i].level1 = 0;
    }

    rmt_transmit_config_t tx = {};
    ESP_ERROR_CHECK(rmt_transmit(channel, copyEnc, symbols,
                                 steps * sizeof(rmt_symbol_word_t), &tx));
    busy = true;
    return true;
}

void xyMotorDriver::waitDone() {
    if (!busy) return;
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, -1));
    busy = false;
}

void xyMotorDriver::moveSteps(size_t steps, uint32_t freqHz) {
// if freqHz = 400, then 15rpm. RPM = freq_hz × 60 / (full_steps_per_rev × microstepping)
    if (steps == 0 || steps > MAX_STEPS) return;
    uint32_t halfPeriod = (500000 / freqHz);
    ESP_LOGI(TAG, "halfPeriod: = %lu", halfPeriod);
    for (size_t i = 0; i < steps; i++) {
        symbols[i].duration0 = static_cast<uint16_t>(halfPeriod);
        symbols[i].level0 = 1;
        symbols[i].duration1 = static_cast<uint16_t>(halfPeriod);
        symbols[i].level1 = 0;
    };
    rmt_transmit_config_t tx = {};
    ESP_ERROR_CHECK(rmt_transmit(channel, copyEnc, symbols, (steps * sizeof(rmt_symbol_word_t)), &tx));
    //steps * sizeof(rmt_symbol_word_t) = how many bytes are being sent
    //ie) steps = 158, each step = 4 bytes, 4 bytes = 32 bit signal
    //rmt_tx_wait_all_done() sends a signal back when it's executed and blocks until it's complete
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(channel, -1));  //make non blocking with ATTR ITRM function
}

esp_err_t xyMotorDriver::onOff(uint8_t toggle) { //when used must be in ESP_ERROR_CHECK()
    ESP_RETURN_ON_FALSE(toggle == 0 || toggle == 1, ESP_ERR_INVALID_ARG, TAG,
                         "Toggle must be 0 or 1, got %d", toggle);
    gpio_set_level(enaPin, toggle);
    return ESP_OK;
}
esp_err_t xyMotorDriver::dir(uint8_t dir) {
    ESP_RETURN_ON_FALSE(dir == 0 || dir == 1, ESP_ERR_INVALID_ARG, TAG,
                            "Direction must be 0 or 1, got %d", dir);
    gpio_set_level(dirPin, dir);
    return ESP_OK;
}




}