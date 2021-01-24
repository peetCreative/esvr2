//
// Created by peetcreative on 11.01.21.
//
#include "Esvr2Opt1Controller.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2GameState.h"
#include "Esvr2LaparoscopeController.h"

#include <boost/bind.hpp>

namespace esvr2
{
    Opt1Controller::Opt1Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState) :
        Controller(laparoscopeController, gameState)
    {
        //Create new device
        gameState->createInteractiveElement2D(
                "Opt1Left",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_LEFT));
        gameState->createInteractiveElement2D(
                "Opt1Right",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_RIGHT));
        gameState->createInteractiveElement2D(
                "Opt1RollLeft",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_ROLL_LEFT));
        gameState->createInteractiveElement2D(
                "Opt1RollRight",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_ROLL_RIGHT));
        gameState->createInteractiveElement2D(
                "Opt1Up",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_UP));
        gameState->createInteractiveElement2D(
                "Opt1Down",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_DOWN));
        gameState->createInteractiveElement2D(
                "Opt1TransIn",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_TRANS_IN));
        gameState->createInteractiveElement2D(
                "Opt1TransOut",
                (boost::function<void()>) 0,
                boost::bind(&Opt1Controller::holdBtn, this, _1, DIR_TRANS_OUT));
    }

    void Opt1Controller::holdBtn(Ogre::uint64 since, DirectionType dir)
    {
        Ogre::Quaternion headOrientationWORLD =
                mGameState->getHeadOrientation();
        Ogre::Quaternion toScreenOrientationWORLD =
                mGameState->getProjectionPlanesOrientation();
        Ogre::Quaternion headOrientationRel =
//                headOrientationWORLD;
                headOrientationWORLD * toScreenOrientationWORLD.Inverse();

        LaparoscopeDOFBoundaries boundaries;
        LaparoscopeDOFPose pose;
        if (!mLaparoscopeController->getLaparoscopeBoundaries(boundaries) ||
            !mLaparoscopeController->getLaparoscopePose(pose))
        {
            LOG << "In Move mode but did not get DOFPose or DOFBoundaries" << LOGEND;
            return;
        }

        float inc = (boundaries.yawMax - boundaries.yawMin) / 100;
        if (dir == DIR_LEFT)
            pose.yaw += inc;
        if (dir == DIR_RIGHT)
            pose.yaw -= inc;

        //rotation around -z
        inc = (boundaries.rollMax - boundaries.rollMin) / 100;
        if (dir == DIR_ROLL_LEFT)
            pose.roll -= inc;
        if (dir == DIR_ROLL_RIGHT)
            pose.roll += inc;

        inc = (boundaries.pitchMax - boundaries.pitchMin) / 100;
        if (dir == DIR_UP)
            pose.pitch += inc;
        if (dir == DIR_DOWN)
            pose.pitch -= inc;

        inc =  (boundaries.transZMax - boundaries.transZMin)/100;
        if (dir == DIR_TRANS_IN)
            pose.transZ += inc;
        if (dir == DIR_TRANS_OUT)
            pose.transZ -= inc;

        pose.yaw = std::min(pose.yaw, boundaries.yawMax);
        pose.yaw = std::max(pose.yaw, boundaries.yawMin);
        pose.pitch = std::min(pose.pitch, boundaries.pitchMax);
        pose.pitch = std::max(pose.pitch, boundaries.pitchMin);
        pose.roll = std::min(pose.roll, boundaries.rollMax);
        pose.roll = std::max(pose.roll, boundaries.rollMin);
        pose.transZ = std::min(pose.transZ, boundaries.transZMax);
        pose.transZ = std::max(pose.transZ, boundaries.transZMin);
        mLaparoscopeController->moveLaparoscopeTo(pose);
    }
}
