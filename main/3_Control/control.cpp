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
                 logic::xyMotorLogic& shoulder, logic::xyMotorLogic& elbow,
                 const Config& cfg)
    : stick_(stick), pos_(pos), shoulder_(shoulder), elbow_(elbow), cfg_(cfg) {
    const float twoPi = 6.28318530718f;
    stepsPerRad1_ = (360.0f / cfg_.stepAngleDeg) * cfg_.microstep1 * cfg_.gear1 / twoPi;
    stepsPerRad2_ = (360.0f / cfg_.stepAngleDeg) * cfg_.microstep2 * cfg_.gear2 / twoPi;
}
//convert tip position to angular motor steps
bool Control::toSteps(float x, float y, long& s1, long& s2) const {
    float a1, a2;
    if (!pos_.ik(x, y, a1, a2)) return false;
    float m2 = cfg_.elbowAbsolute ? (a1 + a2) : a2;
    s1 = lroundf(a1 * stepsPerRad1_);
    s2 = lroundf(m2 * stepsPerRad2_);
    return true;
}

void Control::begin() {
    shoulder_.enable(0);   // 0 = enabled on your drivers (same as your original main)
    elbow_.enable(0);

    tx_ = cfg_.startX;
    ty_ = cfg_.startY;
    if (!toSteps(tx_, ty_, steps1_, steps2_)) {
        ESP_LOGE(TAG, "Home (%.2f, %.2f) is outside the reachable workspace [%.2f, %.2f]",
                 tx_, ty_, pos_.minReach(), pos_.maxReach());
        ready_ = false;
        return;
    }
    ready_ = true;
    ESP_LOGI(TAG, "Ready. Home (%.2f, %.2f) -> steps (%ld, %ld)", tx_, ty_, steps1_, steps2_);
}

// Try to move the tip by (dx, dy). If either motor would need more than maxStepsPerSeg,
// shrink the whole move (both axes together) so the direction stays the same.
bool Control::plan(float dx, float dy, Plan& out) const {
    if (dx == 0.0f && dy == 0.0f) return false;

    for (int attempt = 0; attempt < 4; attempt++) {
        float nx = tx_ + dx;
        float ny = ty_ + dy;
        if (ny < cfg_.yMin) return false;

        long s1, s2;
        if (!toSteps(nx, ny, s1, s2)) return false;

        long m = std::max(labs(s1 - steps1_), labs(s2 - steps2_));
        if (m <= static_cast<long>(cfg_.maxStepsPerSeg)) {
            out = {nx, ny, s1, s2};
            return true;
        }
        float k = 0.9f * static_cast<float>(cfg_.maxStepsPerSeg) / static_cast<float>(m);
        dx *= k;
        dy *= k;
    }
    return false;
}
//one 20ms movement sequence
void Control::tick() {
    const TickType_t segTicks = std::max<TickType_t>(1, pdMS_TO_TICKS(cfg_.segmentUs / 1000));

    if (!ready_) {
        vTaskDelay(segTicks);
        return;
    }

    // 1. Joystick -> unit vector in tip space
    float jx = stick_.x();
    float jy = stick_.y();
    if (cfg_.joySwapXY) std::swap(jx, jy);
    if (cfg_.joyInvertX) jx = -jx;
    if (cfg_.joyInvertY) jy = -jy;
    float mag = hypotf(jx, jy);
    if (mag > 1.0f) { jx /= mag; jy /= mag; }   // square gate: keep diagonals from being faster

    // 2. Cartesian displacement for this segment
    const float dt = static_cast<float>(cfg_.segmentUs) * 1e-6f;
    float dx = jx * cfg_.vMax * dt;
    float dy = jy * cfg_.vMax * dt;

    // 3. Plan it. If the full move is blocked (workspace edge), slide along each axis instead.
    Plan p{};
    bool moved = plan(dx, dy, p) || plan(dx, 0.0f, p) || plan(0.0f, dy, p);

    long d1 = 0, d2 = 0;
    if (moved) {
        d1 = p.s1 - steps1_;
        d2 = p.s2 - steps2_;
        tx_ = p.x;          // target always advances, even if this segment rounds to 0 steps
        ty_ = p.y;
        steps1_ = p.s1;
        steps2_ = p.s2;
    }
    static int n = 0;
    if (++n % 12 == 0) {   // about 4 times per second
        ESP_LOGI(TAG, "stick(%.2f,%.2f) tip(%.2f,%.2f) steps(%ld,%ld) d(%ld,%ld) moved=%d",
                 jx, jy, tx_, ty_, steps1_, steps2_, d1, d2, moved);
    }
    if (d1 == 0 && d2 == 0) {
        vTaskDelay(segTicks);
        return;
    }

    // 4. Drive both joints for the same duration so they finish together => straight line
    bool dir1 = (d1 >= 0) != cfg_.invertShoulder;
    bool dir2 = (d2 >= 0) != cfg_.invertElbow;
    shoulder_.setDir(dir1 ? 1 : 0);
    elbow_.setDir(dir2 ? 1 : 0);
    esp_rom_delay_us(10);   // DIR setup time before the first pulse

    shoulder_.start(static_cast<uint32_t>(labs(d1)), cfg_.segmentUs);
    elbow_.start(static_cast<uint32_t>(labs(d2)), cfg_.segmentUs);
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
