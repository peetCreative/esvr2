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
        Ogre::uint64 mTimeSinceLast;
        const Ogre::uint64 mDelay = 100;
        bool mBlocked = false;
    public:
        Opt0Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);
        void keyPressed();
        void holdPressed(Ogre::uint64 timesincelast);
    };
}


#endif //ESVR2_ESVR2OPT0CONTROLLER_H
