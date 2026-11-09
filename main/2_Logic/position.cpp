// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <cmath>
#include "position.hpp"
#include "esp_err.h"




namespace logic {
    Position::Position(float l1, float l2, bool elbowDown)
        : l1_(l1), l2_(l2), elbowDown_(elbowDown) {}

    bool Position::ik(float x, float y, float& th1, float& th2) const {
        float c2 = (x * x + y * y - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_);
        if (!(c2 >= -C2_LIMIT && c2 <= C2_LIMIT)) return false;  // also rejects NaN

        float a2 = acosf(c2);
        if (!elbowDown_) a2 = -a2;
        th2 = a2;
        th1 = atan2f(y, x) - atan2f(l2_ * sinf(a2), l1_ + l2_ * cosf(a2));
        return true;
    }

    void Position::fk(float th1, float th2, float& x, float& y) const {
        x = l1_ * cosf(th1) + l2_ * cosf(th1 + th2);
        y = l1_ * sinf(th1) + l2_ * sinf(th1 + th2);
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
