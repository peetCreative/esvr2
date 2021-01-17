//
// Created by peetcreative on 13.12.20.
//
#include "Esvr2Opt0Controller.h"
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2GameState.h"


#include "SDL.h"


namespace esvr2
{
    Opt0Controller::Opt0Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController,
            GameState *gameState):
            Controller(laparoscopeController, gameState),
            mMoveMode(false)
    {
    }

    bool Opt0Controller::keyPressed(const SDL_KeyboardEvent &arg )
    {
//        if (arg.keysym.scancode == SDL_SCANCODE_M )
//        {
//            mMoveMode = true;
//        }
        return false;
    }
    bool Opt0Controller::keyReleased(const SDL_KeyboardEvent &arg )
    {
        bool succ = false;
        if (arg.keysym.scancode == SDL_SCANCODE_M )
        {
            //Toggle
                mMoveMode = !mMoveMode;
        }
        return succ;
    }

    void Opt0Controller::headPoseUpdated()
    {
        if (!mMoveMode)
        {
            mGameState->setDebugText("");
            return;
        }
        Ogre::Quaternion headOrientationWORLD =
                mGameState->getHeadOrientation();
        Ogre::Quaternion toScreenOrientationWORLD =
                mGameState->getProjectionPlanesOrientation();
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
        Ogre::Radian pitchRad = headOrientationRel.getPitch();
        Ogre::Degree pitchDeg(pitchRad);
        // swingy seems to be inverted
        if (Ogre::Degree(5) < pitchDeg && pitchDeg < Ogre::Degree(10))
            pose.swingY -= 0.01;
        if (Ogre::Degree(-5) > pitchDeg && pitchDeg > Ogre::Degree(-10))
            pose.swingY += 0.01;

        Ogre::Radian yawRad(headOrientationRel.getYaw());
        Ogre::Degree yawDeg(yawRad);
        if (Ogre::Degree(5) < yawDeg && yawDeg < Ogre::Degree(15))
            pose.swingX += 0.01;
        if (Ogre::Degree(-5) > yawDeg && yawDeg > Ogre::Degree(-15))
            pose.swingX -= 0.01;

        pose.swingX = std::min(pose.swingX, boundaries.swingXMax);
        pose.swingX = std::max(pose.swingX, boundaries.swingXMin);
        pose.swingY = std::min(pose.swingY, boundaries.swingYMax);
        pose.swingY = std::max(pose.swingY, boundaries.swingYMin);
        mLaparoscopeController->moveLaparoscopeTo(pose);
        Ogre::String debugText = "mMoveMode ";
        debugText += "Pitch Rad: " + Ogre::StringConverter::toString(pitchRad) + "\n";
        debugText += "Pitch Deg: " + Ogre::StringConverter::toString(pitchDeg) + "\n";
        debugText += "Yaw Rad: " + Ogre::StringConverter::toString(yawRad) + "\n";
        debugText += "Yaw Deg: " + Ogre::StringConverter::toString(yawDeg) + "\n";
        mGameState->setDebugText(debugText);
    }


}