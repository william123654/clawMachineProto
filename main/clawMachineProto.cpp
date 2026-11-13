// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>

#include "joyStickDriver.hpp"
#include "joyStickLogic.hpp"
#include "xyMotorDriver.hpp"
#include "xyMotorLogic.hpp"
#include "position.hpp"
#include "control.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_rom_sys.h"

//NOTE: inline constexpr.... creates one of the constant value that is referenced in every file that #includes the file
// with that constexpr. if you use just constexpr.... then every file that #includes that fie with have it's own
//version/copy/thing of that constexpr.

constexpr gpio_num_t enaPinX = GPIO_NUM_18;
constexpr gpio_num_t dirPinX = GPIO_NUM_21;
constexpr gpio_num_t pulsePinX = GPIO_NUM_5;
constexpr gpio_num_t enaPinY = GPIO_NUM_23;  //change
constexpr gpio_num_t dirPinY = GPIO_NUM_22; //change
constexpr gpio_num_t pulsePinY = GPIO_NUM_4; //change
constexpr gpio_num_t jPinX = GPIO_NUM_33;
constexpr gpio_num_t jPinY = GPIO_NUM_35;
constexpr gpio_num_t jPinB = GPIO_NUM_26;


extern "C" void app_main(void)
{

    // static: these hold big buffers / must outlive app_main's stack frame
    static driver::xyMotorDriver shoulderDrv(enaPinX, dirPinX, pulsePinX);  // motor at the origin
    static driver::xyMotorDriver elbowDrv(enaPinY, dirPinY, pulsePinY);     // motor at the joint between l1 and l2
    static driver::JoyStickDriver stickDrv;
    stickDrv.init();

    static logic::JoyStickLogic stick(stickDrv);
    stick.calibrate();                      // keep your hands off the stick for ~150 ms
    for (int i = 0; i < 100; i++) {
        printf("raw %d %d  norm %.2f %.2f\n",
               stickDrv.getRawX(), stickDrv.getRawY(), stick.x(), stick.y());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    static logic::xyMotorLogic shoulder(shoulderDrv);
    static logic::xyMotorLogic elbow(elbowDrv);
    static logic::Position pos(26.5f, 26.5f); // l1, l2
    for (int i = 0; i < 200; i++) {
        printf("%d \n",stickDrv.getRawY());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    control::Config cfg;
    cfg.invertShoulder = true;
    cfg.invertElbow = false;
    cfg.joyInvertX = false;
    cfg.joyInvertY = true; // if the tip moves opposite to the stick
    cfg.joySwapXY = true;
    cfg.vMax = 3;
    static control::Control ctl(stick, pos, shoulder, elbow, cfg);
    ctl.begin();
    ctl.run();   // never returns

}
