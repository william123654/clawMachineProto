// clawMachineProto
// Copyright 2026 UCO Robotics Club Contributers
// SPDX-FileCopyrightText: 2026 Zach Beau and UCO Robotics Club contributors
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include "joyStickLogic.hpp"
#include "position.hpp"
#include "xyMotorLogic.hpp"


namespace control {

struct Config {

    /*
    * start pose (startX, startY): these are in the same units as l1 and l2, and the point has to be reachable.
    * The default (0, 7) only works if |l1 - l2| < 7 < l1 + l2 (roughly). If it’s outside that ring, begin()
    * logs an error and the arm won’t run. The arm also has to be physically placed at that pose at power-up.
    * yMin: it was chosen for a 4/6 arm. Set it to wherever the tip shouldn’t go below, such as the table surface.
    * vMax: it’s in length-units per second, so if the new arm is much bigger or smaller (or you changed units),
    * the same number gives a very different real speed.
    *
    *
    * Reachable positions by the arm: maxY value is 2.65 < x < 52.93
     */


    // tip start position
    float startX = 0.0f;
    float startY = 40.0f;

    float vMax = 2.0f;                  //length units/second
                                        //lower if motor skips under load
    float yMin = 2.65f;                 // keeps tip above this line

    uint32_t segmentUs = 20000;         // one straight-line segment = 20 ms
    uint32_t maxStepsPerSeg = 64;       // per motor; 16 steps / 20 ms = 800 Hz. must be <= MAX_STEPS
                                        //microstepping 1x @ maxStepsPerSeg = 16 is speed
                                        //microstepping 8x @ maxStepsPerSeg = 16 is speed / 8 (8x slower)
                                        //microstepping 8x @ maxStesPerSeg = 128 is = to speed @ 1x


    // Motor / mechanics. Set according to dip switch
    float stepAngleDeg = 1.8f; //1.8 is confirmed
    int   microstep1 = 4, microstep2 = 4;
    float gear1 = 3.0f, gear2 = 1.0f; //gear1, microstep1 are tied to stepsPerRad1_, one motor shoulder or elbow
                                        //same for microstep2, gear2
                                        //gear = driven wheel / motor wheel
                                        //diameter of wheel measured at the belt contact point + belt width

    bool invertShoulder = false;        // flip if the joint rotates the wrong way
    bool invertElbow = false;
    bool elbowAbsolute = false;         // true if motor 2 sits at the base and drives link 2 via belt/linkage

    // Joystick orientation
    bool joySwapXY = false;
    bool joyInvertX = false;
    bool joyInvertY = false;
};

class Control {
    logic::JoyStickLogic& stick_;
    logic::Position& pos_;
    logic::xyMotorLogic& shoulder_;
    logic::xyMotorLogic& elbow_;
    Config cfg_;

    float stepsPerRad1_, stepsPerRad2_;
    float tx_ = 0, ty_ = 0;             // where the tip is supposed to be, commanded position
    long steps1_ = 0, steps2_ = 0;      // absolute step counters (relative to home pose)
    bool ready_ = false;                //false if home pose was invalid. tick() will do nothing

    //one proposed move
    struct Plan {
        float x, y;
        long s1, s2;
    };

    bool toSteps(float x, float y, long& s1, long& s2) const;
    bool plan(float dx, float dy, Plan& out) const;

public:
    Control(logic::JoyStickLogic& stick, logic::Position& pos,
            logic::xyMotorLogic& shoulder, logic::xyMotorLogic& elbow,
            const Config& cfg = Config{});

    void begin();   // enable motors and set the home pose
    void tick();    // runs one segment (blocks ~segmentUs)
    void run();     // tick() forever

    float tipX() const { return tx_; }
    float tipY() const { return ty_; }
};
}
