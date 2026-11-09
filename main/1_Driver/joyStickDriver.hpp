// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "../2_Logic/direction.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "esp_adc/adc_oneshot.h"

namespace driver {

class JoyStickDriver {
    int xRaw, yRaw;
    adc_channel_t xJoy = ADC_CHANNEL_4;//GPIO_NUM_32
    adc_channel_t yJoy = ADC_CHANNEL_7;//GPIO_NUM_35
    gpio_num_t bPin;
    adc_oneshot_unit_handle_t joyStick = nullptr;

public:
    JoyStickDriver();
    void init();
    bool isPushed();
    int getRawX();
    int getRawY();
};


}