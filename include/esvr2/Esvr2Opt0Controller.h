//
// Created by peetcreative on 13.12.20.
//

#ifndef ESVR2_ESVR2OPT0CONTROLLER_H
#define ESVR2_ESVR2OPT0CONTROLLER_H
#include "Esvr2Controller.h"

#include "OgreQuaternion.h"
#include "OgreVector3.h"
namespace esvr2
{
    class Opt0Controller : public Controller
    {
    private:
        Ogre::Vector3 mStartPosition = Ogre::Vector3::ZERO;
        Ogre::Quaternion mStartOrientation = Ogre::Quaternion::ZERO;
        Ogre::uint64 mTimeSinceLast;
        Ogre::uint64 mDelay;
        bool mBlocked = false;

        Ogre::Real mStepYaw;
        Ogre::Real mStepPitch;
        Ogre::Real mStepRoll;
        Ogre::Real mStepTransZ;
        Ogre::Real mThresholdTransZ;
        Ogre::Real mThresholdYawDeg;
        Ogre::Real mThresholdPitchDeg;
        Ogre::Real mThresholdRollDeg;
    public:
        Opt0Controller(
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
                Ogre::Real thresholdRollDeg);
        void keyPressed();
        void holdPressed(Ogre::uint64 timesincelast);
        std::string getControllerMenuId() override;
    };
}


#endif //ESVR2_ESVR2OPT0CONTROLLER_H
