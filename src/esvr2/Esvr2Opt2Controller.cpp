//
// Created by peetcreative on 11.01.21.
//
#include "Esvr2Opt2Controller.h"
#include "Esvr2GameState.h"
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"

#include <boost/bind.hpp>

namespace esvr2
{
    Opt2Controller::Opt2Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState) :
        Controller(laparoscopeController, gameState)
    {
        gameState->createInteractiveElement2D(
                "Opt2Move",
                boost::bind(&Opt2Controller::startMoving, this),
                boost::bind(&Opt2Controller::hold, this, _1));
    }

    void Opt2Controller::startMoving()
    {
        mStartOrientation = mGameState->getHeadOrientation();
        mStartPosition    = mGameState->getHeadPosition();
    }

    void Opt2Controller::hold(Ogre::uint64 time)
    {

    }

}