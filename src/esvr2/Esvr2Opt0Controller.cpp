//
// Created by peetcreative on 13.12.20.
//
#include "Esvr2Opt0Controller.h"
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2GameState.h"

#include "boost/bind.hpp"
#include "SDL.h"

#define MENU_OPT0 "MenuOpt0"

namespace esvr2
{
    Opt0Controller::Opt0Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState,
            Ogre::uint64 delay,
            Ogre::Real stepYaw,
            Ogre::Real stepPitch,
            Ogre::Real stepRoll,
            Ogre::Real stepTransZ,
            Ogre::Real thresholdTransZ,
            Ogre::Real thresholdYawDeg,
            Ogre::Real thresholdPitchDeg,
            Ogre::Real thresholdRollDeg):
        Controller(laparoscopeController, gameState),
        mDelay(delay),
        mStepYaw(stepYaw),
        mStepPitch(stepPitch),
        mStepRoll(stepRoll),
        mStepTransZ(stepTransZ),
        mThresholdTransZ(thresholdTransZ),
        mThresholdYawDeg(thresholdYawDeg),
        mThresholdPitchDeg(thresholdPitchDeg),
        mThresholdRollDeg(thresholdRollDeg)
    {
        mGameState->createInteractiveElement2D("CenterButton",
               boost::bind(&Opt0Controller::keyPressed, this),
               boost::bind(&Opt0Controller::holdPressed, this, _1),
               {Ogre::IdString(MENU_OPT0)});
    }

    void Opt0Controller::keyPressed()
    {
        mTimeSinceLast = 0;
        mStartOrientation = mGameState->getHeadOrientation();
        mStartPosition = mGameState->getHeadPosition();
        mBlocked = !mGameState->isHeadPositionCentered();
        if (mBlocked)
            mGameState->setDebugText("too far from center");
        else
            mGameState->setDebugText("");
    }

    void Opt0Controller::holdPressed(Ogre::uint64 timesincelast)
    {
        if (mBlocked || mTimeSinceLast + mDelay >  timesincelast)
        {
            return;
        }
        mTimeSinceLast = timesincelast;

        Ogre::Vector3 posDiff = mStartPosition -
                mGameState->getHeadPosition();
        Ogre::Quaternion headOrientationWORLD =
                mGameState->getHeadOrientation();
        Ogre::Quaternion toScreenOrientationWORLD =
                mGameState->getProjectionPlanesOrientation();
        Ogre::Vector3 axis = toScreenOrientationWORLD.xAxis();
        Ogre::Real headPositionRel = posDiff.length() * axis.dotProduct(posDiff);
        Ogre::Quaternion headOrientationRel =
//                headOrientationWORLD;
            headOrientationWORLD * mStartOrientation.Inverse();
        //TODO: guard
        LaparoscopeDOFBoundaries boundaries;
        LaparoscopeDOFPose pose;
        if (!mLaparoscopeController->getLaparoscopeBoundaries(boundaries) ||
                !mLaparoscopeController->getLaparoscopePose(pose))
        {
            LOG << "In Move mode but did not get DOFPose or DOFBoundaries" << LOGEND;
            return;
        }

        if (headPositionRel > mThresholdTransZ)
            pose.transZ += mStepTransZ;
        if (headPositionRel < -mThresholdTransZ)
            pose.transZ -= mStepTransZ;
        Ogre::Radian pitchRad = headOrientationRel.getPitch();
        Ogre::Degree pitchDeg(pitchRad);
        // swingy seems to be inverted
        if (Ogre::Degree(mThresholdPitchDeg) < pitchDeg)
            pose.pitch += mStepPitch;
        if (Ogre::Degree(-mThresholdPitchDeg) > pitchDeg)
            pose.pitch -= mStepPitch;

        Ogre::Radian yawRad(headOrientationRel.getYaw());
        Ogre::Degree yawDeg(yawRad);
        if (Ogre::Degree(mThresholdYawDeg) < yawDeg)
            pose.yaw += mStepYaw;
        if (Ogre::Degree(-mThresholdYawDeg) > yawDeg)
            pose.yaw -= mStepYaw;

        Ogre::Radian rollRad(headOrientationRel.getRoll());
        Ogre::Degree rollDeg(rollRad);
        if (Ogre::Degree(mThresholdRollDeg) < rollDeg)
            pose.roll += mStepRoll;
        if (Ogre::Degree(-mThresholdRollDeg) > rollDeg)
            pose.roll -= mStepRoll;

        pose.yaw = std::min(pose.yaw, boundaries.yawMax);
        pose.yaw = std::max(pose.yaw, boundaries.yawMin);
        pose.pitch = std::min(pose.pitch, boundaries.pitchMax);
        pose.pitch = std::max(pose.pitch, boundaries.pitchMin);
        pose.roll = std::min(pose.roll, boundaries.rollMax);
        pose.roll = std::max(pose.roll, boundaries.rollMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);
        mLaparoscopeController->moveLaparoscopeTo(pose);
//        Ogre::String debugText = "mMoveMode ";
//        debugText += "Trans Z: " + Ogre::StringConverter::toString(headPositionRel) + "\n";
//        debugText += "Pitch Rad: " + Ogre::StringConverter::toString(pitchRad) + "\n";
//        debugText += "Pitch Deg: " + Ogre::StringConverter::toString(pitchDeg) + "\n";
//        debugText += "Yaw Rad: " + Ogre::StringConverter::toString(yawRad) + "\n";
//        debugText += "Yaw Deg: " + Ogre::StringConverter::toString(yawDeg) + "\n";
//        mGameState->setDebugText(debugText);
    }

    std::string Opt0Controller::getControllerMenuId() {
        return MENU_OPT0;
    }
}