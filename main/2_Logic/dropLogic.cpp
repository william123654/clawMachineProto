
#include "dropLogic.hpp"
#include "zMotorDriver.hpp"
#include "xyMotorDriver.hpp"
#include "xyMotorLogic.hpp"

namespace Logic{
    dropLogic::dropLogic(xyMotorDriver& s, xyMotorDriver& e, zMotorDriver z) 
        : shoulder(s), elbow(e), drop(z) {

    };

    void dropLogic::drop() {
        
    };
}
