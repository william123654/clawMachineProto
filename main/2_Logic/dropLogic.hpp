#pragma once

#include "zMotorDriver.hpp"
#include "xyMotorDriver.hpp"

namespace Logic {

    class dropLogic {
        driver::xyMotorDriver shoulder;
        driver::xyMotorDriver elbow;
        driver::zMotorDriver drop;

        struct Plan {
        float x, y;
        long s1, s2;
        };

        public:
        dropLogic(xyMotorDriver& s, xyMotorDriver& e, zMotorDriver& z);
        void drop();
        void plan(float x, float y, Plan& out);
    }
    


}