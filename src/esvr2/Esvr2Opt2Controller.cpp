//
// Created by peetcreative on 11.01.21.
//
#include "Esvr2Opt2Controller.h"
#include "Esvr2GameState.h"
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"

#include "PivotControlMessages.h"

#include <cmath>
#include <boost/bind.hpp>

#define MENU_OPT2 "MenuOpt2"

using namespace pivot_control_messages;

namespace esvr2
{
    Opt2Controller::Opt2Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState,
            Ogre::Real transZFact,
            Ogre::Real cameraTilt,
            Ogre::Real focusDistance) :
        Controller(laparoscopeController, gameState),
        mTransZFact(transZFact)
    {
        mTogglePressCallback = boost::bind(&Opt2Controller::startMoving, this);
        mHoldCallback = boost::bind(&Opt2Controller::hold, this, _1);
        mCamereaTilt = cameraTilt;
        mFocusDistance = focusDistance;
    }

    void Opt2Controller::startMoving()
    {
        mStartOrientation = mGameState->getHeadOrientation();
        mStartPosition = mGameState->getHeadPosition();
        //TODO: guard
        if (!mLaparoscopeController->getCurrentDOFPose(mStartPose))
            return;
//        mBlocked = !mGameState->isHeadPositionCentered();
//        if (mBlocked)
//            mGameState->setDebugText("too far from center");
//        else
//            mGameState->setDebugText("");
    }

    void Opt2Controller::hold(Ogre::uint64 time)
    {
        if (mBlocked)
            return;
        Ogre::Quaternion currentOrientation = mGameState->getHeadOrientation();
        Ogre::Vector3 currentPosition = mGameState->getHeadPosition();
        DOFBoundaries boundaries;
        DOFPose pose = mStartPose;

        if (!mLaparoscopeController->getDOFBoundaries(boundaries) )
        {
            LOG << "In Move mode but did not get DOFBoundaries" << LOGEND;
            return;
        }
        Ogre::Vector3 xAxisTrans = currentOrientation.xAxis();
        if (xAxisTrans == Ogre::Vector3::UNIT_Y &&
            xAxisTrans == -Ogre::Vector3::UNIT_Y)
            return;
        Ogre::Vector3 xAxisNew(xAxisTrans.x,0,xAxisTrans.z);
        xAxisNew.normalise();
        Ogre::Quaternion trans = xAxisTrans.getRotationTo(xAxisNew);
        Ogre::Quaternion currentOrientationAdj = trans * currentOrientation;

        Ogre::Radian pitchDiff =
                mStartOrientation.getPitch() - currentOrientationAdj.getPitch();
        Ogre::Radian yawDiff =
                mStartOrientation.getYaw() - currentOrientationAdj.getYaw();
        Ogre::Radian rollDiff =
                mStartOrientation.getRoll() - currentOrientationAdj.getRoll();
        Ogre::Vector3 zAxis = currentOrientation.zAxis();
        Ogre::Vector3 posDiff = mStartPosition - currentPosition;

        pose.pitch = mStartPose.pitch - pitchDiff.valueRadians();
        pose.yaw =  mStartPose.yaw - yawDiff.valueRadians();
        pose.roll = mStartPose.roll + rollDiff.valueRadians();
        pose.transZ = mStartPose.transZ +
                mTransZFact * (posDiff.length() * zAxis.dotProduct(posDiff));

        if (mEnableTransPitch)
        {
            Ogre::Real transPitch = std::asin(
                    (std::sin(M_PI - mCamereaTilt) * mStartPose.transZ)/
                    mFocusDistance);
            Ogre::Real transPitchNew = std::asin(
                    (std::sin(M_PI - mCamereaTilt) * pose.transZ)/
                    mFocusDistance);
            pose.pitch -= transPitchNew - transPitch;
        }

        pose.yaw = std::min(pose.yaw, boundaries.yawMax);
        pose.yaw = std::max(pose.yaw, boundaries.yawMin);
        pose.pitch = std::min(pose.pitch, boundaries.pitchMax);
        pose.pitch = std::max(pose.pitch, boundaries.pitchMin);
        pose.roll = std::min(pose.roll, boundaries.rollMax);
        pose.roll = std::max(pose.roll, boundaries.rollMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);

        mLaparoscopeController->setTargetDOFPose(pose);
        if(mFollowMovements)
        {
            mGameState->moveScreen(0);
        }
    }

    std::string Opt2Controller::getControllerMenuId() {
        return MENU_OPT2;
    }

}