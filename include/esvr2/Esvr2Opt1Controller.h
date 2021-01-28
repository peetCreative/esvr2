//
// Created by peetcreative on 11.01.21.
//

#ifndef ESVR2_OPT1CONTROLLER_H
#define ESVR2_OPT1CONTROLLER_H

#include "Esvr2Controller.h"
#include "Esvr2InteractiveElement2D.h"
#include "Ogre.h"

namespace esvr2
{
    enum DirectionType
    {
        DIR_LEFT,
        DIR_RIGHT,
        DIR_ROLL_LEFT,
        DIR_ROLL_RIGHT,
        DIR_UP,
        DIR_DOWN,
        DIR_TRANS_IN,
        DIR_TRANS_OUT
    };

    class Opt1Controller: public Controller
    {
    private:
        InteractiveElement2DPtr mDownButton;
        InteractiveElement2DPtr mUpButton;
        InteractiveElement2DPtr mLeftButton;
        InteractiveElement2DPtr mRightButton;
        Ogre::uint64 mTimeSinceLast;
        Ogre::uint64 mDelay;
        Ogre::Real mStepYaw;
        Ogre::Real mStepPitch;
        Ogre::Real mStepRoll;
        Ogre::Real mStepTransZ;
    public:
        Opt1Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState,
                Ogre::uint64 delay,
                Ogre::Real stepYaw,
                Ogre::Real stepPitch,
                Ogre::Real stepRoll,
                Ogre::Real stepTransZ);
        void keyPressed();
        void holdBtn(Ogre::uint64 timesincelast, DirectionType dir);
        std::string getControllerMenuId() override;
    };
}

#endif //ESVR2_OPT1CONTROLLER_H
