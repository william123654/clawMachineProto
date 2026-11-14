// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <cmath>
#include "position.hpp"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

static const char* TAG = "CONTROL";

namespace logic {
    Position::Position(float l1, float l2, bool elbowDown)
        : l1_(l1), l2_(l2), elbowDown_(elbowDown) {}

    bool Position::inverseKinematics(float commandedX, float commandedY, float& theta1, float& theta2) const {
        float c2 = (commandedX * commandedX + commandedY * commandedY - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_); //cos theta
        if (!(c2 >= -C2_LIMIT && c2 <= C2_LIMIT)) return false;  // also rejects NaN

        float a2 = acosf(c2);
        if (!elbowDown_) a2 = -a2;
        theta2 = a2;
        theta1 = atan2f(commandedY, commandedX) - atan2f(l2_ * sinf(a2), l1_ + l2_ * cosf(a2));
        return true;
    }

    void Position::forwardKinematics(float theta1, float theta2, float& currentX, float& currentY) const {
        currentX = l1_ * cosf(theta1) + l2_ * cosf(theta1 + theta2);
        currentY = l1_ * sinf(theta1) + l2_ * sinf(theta1 + theta2);
    }

    float Position::minReach() const {
        float c = -C2_LIMIT;
        return sqrtf(l1_ * l1_ + l2_ * l2_ + 2.0f * l1_ * l2_ * c);
    }

    float Position::maxReach() const {
        float c = C2_LIMIT;
        return sqrtf(l1_ * l1_ + l2_ * l2_ + 2.0f * l1_ * l2_ * c);
    }
}
