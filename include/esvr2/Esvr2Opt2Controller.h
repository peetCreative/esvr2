//
// Created by peetcreative on 11.01.21.
//

#ifndef ESVR2_OPT2CONTROLLER_H
#define ESVR2_OPT2CONTROLLER_H
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2GameState.h"

namespace esvr2
{
    class Opt2Controller : public Controller
    {
    private:
        Ogre::Quaternion mStartOrientation = Ogre::Quaternion::ZERO;
        LaparoscopeDOFPose mStartPose;
    public:
        Opt2Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);

        void startMoving();
        void hold(Ogre::uint64 time);
    };
}

#endif //ESVR2_OPT2CONTROLLER_H
