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

        //TODO: guard
        if (!mLaparoscopeController->getLaparoscopePose(mStartPose))
            return;
    }

    void Opt2Controller::hold(Ogre::uint64 time)
    {
        Ogre::Quaternion currentOrientation = mGameState->getHeadOrientation();
        Ogre::Vector3 currentPosition    = mGameState->getHeadPosition();
        LaparoscopeDOFBoundaries boundaries;
        LaparoscopeDOFPose pose = mStartPose;

        if (!mLaparoscopeController->getLaparoscopeBoundaries(boundaries) )
        {
            LOG << "In Move mode but did not get DOFPose or DOFBoundaries" << LOGEND;
            return;
        }
        Ogre::Quaternion diff = (-mStartOrientation) * currentOrientation;
        Ogre::Radian pitch = diff.getPitch();
        Ogre::Radian yaw = diff.getYaw();
//        pose.swingY = mStartPose.swingY - pitch.valueRadians();
        pose.swingX = mStartPose.swingX + yaw.valueRadians();
        mLaparoscopeController->moveLaparoscopeTo(pose);
    }

}