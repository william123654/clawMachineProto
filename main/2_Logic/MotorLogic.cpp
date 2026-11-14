// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include <algorithm>
#include "MotorDriver.hpp"
#include "MotorLogic.hpp"
#include "esp_err.h"
#include "esp_check.h"

namespace logic {

MotorLogic::MotorLogic(driver::MotorDriver& driver) : driver_(driver) {
    driver_.init();
}

    //look at steps.... does it need size_t? or uint16_t?
void MotorLogic::move(size_t steps, uint32_t freqHz) {
    driver_.moveSteps(steps, freqHz);
}

bool MotorLogic::start(uint32_t steps, uint32_t segmentUS) {
    return driver_.startSteps(steps, segmentUS);
}

void MotorLogic::wait() {
    driver_.waitDone();
}

void MotorLogic::moveABunch(size_t steps, uint32_t freqHz, uint8_t dir) {
    size_t maxMove = 500;
    ESP_ERROR_CHECK(driver_.dir(dir));
    while (steps > 0) {
        if (steps > maxMove) {
            move(maxMove, freqHz);
        } else move(steps, freqHz);
        steps -= maxMove;
    }
}

void MotorLogic::setDir(uint8_t direction) {
    driver_.dir(direction);
}

void MotorLogic::enable(uint8_t toggle) {
    driver_.onOff(toggle);
}

}