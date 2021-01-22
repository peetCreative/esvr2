//
// Created by peetcreative on 14.12.20.
//
#include "Esvr2Controller.h"

#include "Esvr2.h"
#include "Esvr2LaparoscopeController.h"

namespace esvr2
{
    Controller::Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState):
            mLaparoscopeController(laparoscopeController),
            mGameState(gameState)
    {
        LaparoscopeDOFPose lapPose = LaparoscopeDOFPose();
        lapPose.yaw = 0;
        lapPose.pitch = 0;
        lapPose.transZ = 0.16;
        lapPose.roll = 0;
        mLaparoscopeController->moveLaparoscopeTo(lapPose);
    }
}