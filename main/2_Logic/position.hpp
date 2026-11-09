// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#pragma once

#include <cstdint>
#include <numbers>
#include "esp_err.h"

namespace logic {

inline constexpr float RAD_TO_DEG = 180.0f / std::numbers::pi_v<float>;

class Position {
    float l1_, l2_;
    bool elbowDown_;
    static constexpr float C2_LIMIT = 0.995f;  // stay off the fully extended / folded singularities

public:
    Position(float l1 = 4.0f, float l2 = 6.0f, bool elbowDown = true);

    // Returns false if (x, y) is outside the reachable ring.
    bool ik(float x, float y, float& th1, float& th2) const;
    void fk(float th1, float th2, float& x, float& y) const;

    float minReach() const;
    float maxReach() const;
    float l1() const { return l1_; }
    float l2() const { return l2_; }
};
}