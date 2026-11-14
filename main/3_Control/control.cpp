
// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char* TAG = "CONTROL";

namespace control {

//control logic:
//stick → unit vector → point on plane (cartesian) (dx,dy) → new target
//→ IK → joint angles → absolute step counts → deltas
//→ both motors pulse for the same 20 ms

Control::Control(logic::JoyStickLogic& stick, logic::Position& pos,
                 logic::MotorLogic& shoulder, logic::MotorLogic& elbow,
                 const Config& cfg)
    : stick_(stick), pos_(pos), shoulder_(shoulder), elbow_(elbow), cfg_(cfg) {
    const float twoPi = 6.28318530718f;
    stepsPerRad1 = (360.0f / cfg_.stepAngleDeg) * cfg_.microstep1 * cfg_.gear1 / twoPi;
    stepsPerRad2 = (360.0f / cfg_.stepAngleDeg) * cfg_.microstep2 * cfg_.gear2 / twoPi;
}
//convert tip position to angular motor steps
bool Control::toSteps(float x, float y, long& s1, long& s2) const {
    float a1, a2;
    if (!pos_.inverseKinematics(x, y, a1, a2)) return false;
    float m2 = cfg_.elbowAbsolute ? (a1 + a2) : a2;
    s1 = lroundf(a1 * stepsPerRad1);
    s2 = lroundf(m2 * stepsPerRad2);
    return true;
}

void Control::begin() {
    shoulder_.enable(0);   // 0 = enabled on your drivers (same as your original main)
    elbow_.enable(0);
    drop_.enable(0)

    tipX = cfg_.startX;
    tipY = cfg_.startY;
    if (!toSteps(tipX, tipY, steps1, steps2)) {
        ESP_LOGE(TAG, "Home (%.2f, %.2f) is outside the reachable workspace [%.2f, %.2f]",
                 tipX, tipY, pos_.minReach(), pos_.maxReach());
        ready = false;
        return;
    }
    ready = true;
    ESP_LOGI(TAG, "Ready. Home (%.2f, %.2f) -> steps (%ld, %ld)", tipX, tipY, steps1, steps2);
}

// Try to move the tip by (dx, dy). If either motor would need more than maxStepsPerSeg,
// shrink the whole move (both axes together) so the direction stays the same.
//segmentMoveX, segmentMoveY -- how much the tip moves this segment
bool Control::plan(float segmentMoveX, float segmentMoveY, Plan& out) const {

    //check if the max distance check slides along the x axis only or if it follows the curved max distance.


    for (int attempt = 0; attempt < 4; attempt++) {
        // Block any axis that would leave the workspace (lets the tip slide along an edge)
      //  if (ty_ + dy < cfg_.yMin || ty_ + dy > cfg_.yMax) dy = 0.0f;
        if (tipX + segmentMoveX < cfg_.xMin || tipX + segmentMoveX > cfg_.xMax) segmentMoveX = 0.0f;
        if (segmentMoveX == 0.0f && segmentMoveY == 0.0f) return false;

        float nextX = tipX + segmentMoveX;
        float nextY = tipY + segmentMoveY;

        long s1, s2;
        if (!toSteps(nextX, nextY, s1, s2)) return false;

        long m = std::max(labs(s1 - steps1), labs(s2 - steps2));
        if (m <= static_cast<long>(cfg_.maxStepsPerSeg)) {
            out = {nextX, nextY, s1, s2};
            return true;
        }
        // Too many steps for one segment: shrink both axes together so direction stays the same
        float shrinkNextSteps = 0.9f * static_cast<float>(cfg_.maxStepsPerSeg) / static_cast<float>(m);
        segmentMoveX *= shrinkNextSteps;
        segmentMoveY *= shrinkNextSteps;
    }
    ESP_LOGI(TAG, "Movement faster than max velocity");
    return false;
}
//one 20ms movement sequence
void Control::tick() {
    const TickType_t segTicks = std::max<TickType_t>(1, pdMS_TO_TICKS(cfg_.segmentUs / 1000));

    if (!ready) {
        vTaskDelay(segTicks);
        return;
    }

    // 1. Joystick -> unit vector in tip space
    float joyStickX = stick_.x();
    float joyStickY = stick_.y();
    if (cfg_.joySwapXY) std::swap(joyStickX, joyStickY);
    if (cfg_.joyInvertX) joyStickX = -joyStickX;
    if (cfg_.joyInvertY) joyStickY = -joyStickY;
    float joyMagnitude = hypotf(joyStickX, joyStickY);
    if (joyMagnitude > 1.0f) { joyStickX /= joyMagnitude; joyStickY /= joyMagnitude; }   // square gate: keep diagonals from being faster

    // 2. displacement for this segment
    const float tickLength = static_cast<float>(cfg_.segmentUs) * 1E-6f; //scientific notation, converting 20ms to seconds
    float requestedX = joyStickX * cfg_.vMax * tickLength; //stick value * vMax * segmentUS(converted to seconds)
    float requestedY = joyStickY * cfg_.vMax * tickLength;

    // 3. Plan it. If the full move is blocked (workspace edge), slide along each axis instead.
    Plan p{};
    bool moved = plan(requestedX, requestedY, p);

    long shoulderSteps = 0, elbowSteps = 0;
    if (moved) {
        shoulderSteps = p.s1 - steps1;
        elbowSteps = p.s2 - steps2;
        tipX = p.x;          // target always advances, even if this segment rounds to 0 steps
        tipY = p.y;
        steps1 = p.s1;
        steps2 = p.s2;
    }
    static int n = 0;
    if (++n % 12 == 0) {   // about 4 times per second
        ESP_LOGI(TAG, "stick(%.2f,%.2f) tip(%.2f,%.2f) steps(%ld,%ld) d(%ld,%ld) moved=%d",
                 joyStickX, joyStickY, tipX, tipY, steps1, steps2, shoulderSteps, elbowSteps, moved);
    }
    if (shoulderSteps == 0 && elbowSteps == 0) {
        vTaskDelay(segTicks);
        return;
    }

    // 4. Drive both joints for the same duration so they finish together => straight line
    bool dir1 = (shoulderSteps >= 0) != cfg_.invertShoulder;
    bool dir2 = (elbowSteps >= 0) != cfg_.invertElbow;
    shoulder_.setDir(dir1 ? 1 : 0);
    elbow_.setDir(dir2 ? 1 : 0);
    esp_rom_delay_us(10);   // DIR setup time before the first pulse in case direction changes

    shoulder_.start(static_cast<uint32_t>(labs(shoulderSteps)), cfg_.segmentUs);
    elbow_.start(static_cast<uint32_t>(labs(elbowSteps)), cfg_.segmentUs);
    shoulder_.wait();
    elbow_.wait();
}
//calls tick in a continous loop
void Control::run() {
    while (true) {
        tick();
    }
}

}
