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
        mBlocked = !mGameState->isHeadPositionCentered();
        if (mBlocked)
            mGameState->setDebugText("too far from center");
        else
            mGameState->setDebugText("");
    }

    void Opt2Controller::hold(Ogre::uint64 time)
    {
        if (mBlocked)
            return;
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
        pose.pitch = mStartPose.pitch + pitchDiff.valueRadians();
        pose.yaw = mStartPose.yaw - yawDiff.valueRadians();
        pose.transZ = mStartPose.transZ -
                mTransZFact * (posDiff.length() * zAxis.dotProduct(posDiff));
        pose.yaw = std::min(pose.yaw, boundaries.yawMax);
        pose.yaw = std::max(pose.yaw, boundaries.yawMin);
        pose.pitch = std::min(pose.pitch, boundaries.pitchMax);
        pose.pitch = std::max(pose.pitch, boundaries.pitchMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);

        mLaparoscopeController->moveLaparoscopeTo(pose);
        mGameState->moveScreen(0);
    }

}