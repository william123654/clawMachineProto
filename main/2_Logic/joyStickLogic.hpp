// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include "joyStickDriver.hpp"
#include "direction.hpp"

namespace logic {

class JoyStickLogic {
    driver::JoyStickDriver& stickDriver_;
    int xCenter, yCenter;
    float deadzone;
    static constexpr int ADC_MAX = 4095;

    float normalize(int raw, int center) const;

public:
    JoyStickLogic(driver::JoyStickDriver& stickDriver);
    //average stick center position
    void calibrate(int samples = 50);
    float x();
    float y();
    DIRECTION getDirection();
    bool isPushed();
};
}