// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include "joyStickLogic.hpp"
#include "esp_log.h"

static const char* TAG = "JoyStickLogic";
namespace logic {

JoyStickLogic::JoyStickLogic(driver::JoyStickDriver& stickDriver)
    : stickDriver_(stickDriver), xCenter(2002), yCenter(1959), deadzone(0.1f) {}

void JoyStickLogic::calibrate(int samples) {
    float calX = 0, calY = 0;
    for (int i = 0; i < samples; i++) {
        calX += stickDriver_.getRawX();
        calY += stickDriver_.getRawY();
    }
    xCenter = static_cast<int>(calX / samples);
    yCenter = static_cast<int>(calY / samples);
}

float JoyStickLogic::normalize(int raw, int center) const {
    float normal = (raw >= center) ? static_cast<float>(raw - center) / static_cast<float>(ADC_MAX - center)
                                    : static_cast<float>(raw - center) / static_cast<float>(center);
    normal = fmaxf(-1.0f, fminf(1.0f, normal));
    if (fabsf(normal) < deadzone) return 0.0f;
    return (normal > 0 ? normal - deadzone : normal + deadzone) / (1.0f - deadzone);
}


float JoyStickLogic::x() {
    return normalize(stickDriver_.getRawX(), xCenter);
}

float JoyStickLogic::y() {
    return normalize(stickDriver_.getRawY(), yCenter);
}

DIRECTION JoyStickLogic::getDirection() {
    float normalX = x();
    float normalY = y();

    if (normalX == 0.0f && normalY == 0.0f) {
        ESP_LOGI(TAG, "CENTER");
        return DIRECTION::CENTER;
    } else if (normalX > 0 && normalY < 0) {
        ESP_LOGI(TAG, "NORTH");
        return DIRECTION::NORTH;
    } else if (normalX > 0 && normalY > 0) {
        ESP_LOGI(TAG, "EAST");
        return DIRECTION::EAST;
    } else if (normalX < 0 && normalY > 0) {
        ESP_LOGI(TAG, "SOUTH");
        return DIRECTION::SOUTH;
    } else if (normalX < 0 && normalY < 0) {
        ESP_LOGI(TAG, "WEST");
        return DIRECTION::WEST;
    } else return DIRECTION::DIRECTION_ERROR;
}

bool JoyStickLogic::isPushed() {
    return stickDriver_.isPushed();
}

}