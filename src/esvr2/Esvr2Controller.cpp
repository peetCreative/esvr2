//
// Created by peetcreative on 14.12.20.
//
#include "Esvr2Controller.h"
#include "Esvr2InteractiveElement.h"
#include "Esvr2LaparoscopeController.h"

#include <boost/bind.hpp>
#include <utility>

namespace esvr2
{
    Controller::Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState):
            InteractiveElement(),
            mLaparoscopeController(std::move(laparoscopeController)),
            mGameState(gameState)
    {
        mToggleReleaseCallback = boost::bind(&Controller::stopMotion, this);
    }

    // this function is called by default to stop motion
    // after finally releasing the left button on the controller
    void Controller::stopMotion()
    {
        //stop the controller from moving
        DOFPose curDofPose;
        if(mLaparoscopeController->getCurrentDOFPose(curDofPose))
            mLaparoscopeController->setTargetDOFPose(curDofPose);
    }
}