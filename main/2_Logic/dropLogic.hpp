#pragma once

#include "zMotorDriver.hpp"
#include "xyMotorDriver.hpp"

namespace Logic {

    class dropLogic {
        driver::xyMotorDriver shoulder;
        driver::xyMotorDriver elbow;
        driver::zMotorDriver drop;

        public:
        dropLogic(xyMotorDriver& s, xyMotorDriver& e, zMotorDriver& z);
        void drop();
    }


}