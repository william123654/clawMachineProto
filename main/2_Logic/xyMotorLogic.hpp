// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "freertos/FreeRTOS.h"
#include "xyMotorDriver.hpp"


namespace logic {


class xyMotorLogic {
    driver::xyMotorDriver& driver_;
public:
    xyMotorLogic(driver::xyMotorDriver& driver);

    //blocking move function
    void move(size_t steps, uint32_t freqHz);
    //non-blocking move function
    bool start(uint32_t steps, uint32_t segmentUs);
    //wait for start() to finish
    void wait();
    void setDir(uint8_t);
    void enable(uint8_t);


    };
}
