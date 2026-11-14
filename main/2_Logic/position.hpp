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
    Position(float l1, float l2, bool elbowDown = true);

    // Returns false if (x, y) is outside the reachable ring.
    bool inverseKinematics(float commandedX, float commandedY, float& theta1, float& theta2) const;
    void forwardKinematics(float theta1, float theta2, float& currentX, float& currentY) const;

    float minReach() const;
    float maxReach() const;
    float l1() const { return l1_; }
    float l2() const { return l2_; }
};
}

/**
Home	    elbowDown Shoulder th1	Elbow th2	Elbow joint sits at
(26.5, 26.5)	true	0°	        +90°	    (26.5, 0)
(26.5, 26.5)	false	90°	        −90°	    (0, 26.5)
(−26.5, 26.5)	true	90°	        +90°	    (0, 26.5)
(−26.5, 26.5)	false	180°	    −90°	    (−26.5, 0)
**/