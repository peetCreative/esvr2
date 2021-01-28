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
        Ogre::Vector3 mStartPosition = Ogre::Vector3::ZERO;
        LaparoscopeDOFPose mStartPose;
        Ogre::Real mTransZFact = 2.0;
        bool mBlocked = false;
    public:
        Opt2Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState,
                Ogre::Real transZFact);

        void startMoving();
        void hold(Ogre::uint64 time);
        std::string getControllerMenuId() override;
    };
}

#endif //ESVR2_OPT2CONTROLLER_H
