//
// Created by peetcreative on 14.12.20.
//
#include "Esvr2Controller.h"

#include <utility>

#include "Esvr2.h"
#include "Esvr2InteractiveElement.h"
#include "Esvr2LaparoscopeController.h"

namespace esvr2
{
    Controller::Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState):
            InteractiveElement(),
            mLaparoscopeController(std::move(laparoscopeController)),
            mGameState(gameState)
    {
        LaparoscopeDOFPose lapPose = LaparoscopeDOFPose();
        lapPose.yaw = 0;
        lapPose.pitch = 0;
        lapPose.transZ = 0;
        lapPose.roll = 0;
    }
}