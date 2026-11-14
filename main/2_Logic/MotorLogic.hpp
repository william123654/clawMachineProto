// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "freertos/FreeRTOS.h"
#include "MotorDriver.hpp"


namespace logic {


class MotorLogic {
    driver::MotorDriver& driver_;
public:
    MotorLogic(driver::MotorDriver& driver);

    //blocking move function
    void move(size_t steps, uint32_t freqHz);
    //non-blocking move function
    bool start(uint32_t steps, uint32_t segmentUs);
    //wait for start() to finish
    void wait();
    void moveABunch(size_t steps, uint32_t freqHz, uint8_t dir);
    void setDir(uint8_t);
    void enable(uint8_t);


    };
}
