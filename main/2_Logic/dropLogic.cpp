
#include "dropLogic.hpp"
#include "zMotorDriver.hpp"
#include "xyMotorDriver.hpp"
#include "xyMotorLogic.hpp"
#include "control.hpp"

namespace Logic{
    dropLogic::dropLogic(xyMotorDriver& s, xyMotorDriver& e, zMotorDriver z) 
        : shoulder(s), elbow(e), drop(z) {

    };

    void dropLogic::plan(float x, float y, Plan& out) {

    };

    void dropLogic::drop(long shoulderSteps, long elbowSteps) {
        //go down
        drop.setDir(1); //flip if wrong direction
        drop.start(Control::Config.dropSteps, Control::Config.SegmentUS);
        drop.wait();
        //close claw

        //go back up
        drop.setDir(0);
        drop.start(Control::Config.dropSteps * -1, Control::Config.SegmentUS);
        drop.wait();

        //go to chute (mirrors the move part of tick())
        Plan{} p;
        //get step position of chute
        plan(Control::Config.chuteX, Control::Config.chuteY, p);
        //find steps to chute
        long ds = shoulderSteps - p.s1;
        long de = elbowSteps - p.s2;
        //move that many steps
        //open claw
    };
}
