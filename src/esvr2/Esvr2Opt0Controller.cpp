//
// Created by peetcreative on 13.12.20.
//
#include "Esvr2Opt0Controller.h"
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2GameState.h"

#include "boost/bind.hpp"
#include "SDL.h"


namespace esvr2
{
    Opt0Controller::Opt0Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState):
            Controller(laparoscopeController, gameState)
    {
        mGameState->createInteractiveElement2D("Opt0Move",
               boost::bind(&Opt0Controller::keyPressed, this),
               boost::bind(&Opt0Controller::holdPressed, this, _1));
    }

    void Opt0Controller::keyPressed()
    {
        mTimeSinceLast = 0;
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
        Ogre::Vector3 zAxis = toScreenOrientationWORLD.zAxis();
        Ogre::Real headPositionRel = posDiff.length() * zAxis.dotProduct(posDiff);
        Ogre::Quaternion headOrientationRel =
//                headOrientationWORLD;
            headOrientationWORLD * toScreenOrientationWORLD.Inverse();
        //TODO: guard
        LaparoscopeDOFBoundaries boundaries;
        LaparoscopeDOFPose pose;
        if (!mLaparoscopeController->getLaparoscopeBoundaries(boundaries) ||
                !mLaparoscopeController->getLaparoscopePose(pose))
        {
            LOG << "In Move mode but did not get DOFPose or DOFBoundaries" << LOGEND;
            return;
        }

        float inc =  (boundaries.transZMax - boundaries.transZMin)/100;
        if (headPositionRel > 0.05)
            pose.transZ -= inc;
        if (headPositionRel < -0.05)
            pose.transZ += inc;
        Ogre::Radian pitchRad = headOrientationRel.getPitch();
        Ogre::Degree pitchDeg(pitchRad);
        // swingy seems to be inverted
        inc =  (boundaries.swingYMax - boundaries.swingYMin)/100;
        if (Ogre::Degree(5) < pitchDeg)
            pose.swingY -= inc;
        if (Ogre::Degree(-5) > pitchDeg)
            pose.swingY += inc;

        Ogre::Radian yawRad(headOrientationRel.getYaw());
        Ogre::Degree yawDeg(yawRad);
        inc =  (boundaries.swingXMax - boundaries.swingXMin)/100;
        if (Ogre::Degree(5) < yawDeg)
            pose.swingX += inc;
        if (Ogre::Degree(-5) > yawDeg)
            pose.swingX -= inc;

        pose.swingX = std::min(pose.swingX, boundaries.swingXMax);
        pose.swingX = std::max(pose.swingX, boundaries.swingXMin);
        pose.swingY = std::min(pose.swingY, boundaries.swingYMax);
        pose.swingY = std::max(pose.swingY, boundaries.swingYMin);
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


}