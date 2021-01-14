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
        mStartPosition = mGameState->getHeadPosition();
        //TODO: guard
        if (!mLaparoscopeController->getLaparoscopePose(mStartPose))
            return;
    }

    void Opt2Controller::hold(Ogre::uint64 time)
    {
        Ogre::Quaternion currentOrientation = mGameState->getHeadOrientation();
        Ogre::Vector3 currentPosition = mGameState->getHeadPosition();
        LaparoscopeDOFBoundaries boundaries;
        LaparoscopeDOFPose pose = mStartPose;

        if (!mLaparoscopeController->getLaparoscopeBoundaries(boundaries) )
        {
            LOG << "In Move mode but did not get DOFPose or DOFBoundaries" << LOGEND;
            return;
        }

        Ogre::Radian pitchDiff =
                mStartOrientation.getPitch() - currentOrientation.getPitch();
        Ogre::Radian yawDiff =
                mStartOrientation.getYaw() - currentOrientation.getYaw();
        Ogre::Vector3 zAxis = currentOrientation.zAxis();
        Ogre::Vector3 posDiff = mStartPosition - currentPosition;
        pose.swingY = mStartPose.swingY + pitchDiff.valueRadians();
        pose.swingX = mStartPose.swingX - yawDiff.valueRadians();
        pose.transZ = mStartPose.transZ -
                mTransZFact * (posDiff.length() * zAxis.dotProduct(posDiff));
        pose.swingX = std::min(pose.swingX, boundaries.swingXMax);
        pose.swingX = std::max(pose.swingX, boundaries.swingXMin);
        pose.swingY = std::min(pose.swingY, boundaries.swingYMax);
        pose.swingY = std::max(pose.swingY, boundaries.swingYMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);

        mLaparoscopeController->moveLaparoscopeTo(pose);
        mGameState->moveScreen(0);
    }

}