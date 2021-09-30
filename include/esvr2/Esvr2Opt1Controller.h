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

    //! \brief Laparoscope Controller inducing a constant movement
    //! in a certain direction when when the interactive elements are held
    /*! \remarks
     * The controller features six interactive elements/ buttons
     * (left, right, up, down, in, out).
     * By holding them a constant motion in this direction
     * is supposed to be induced
     * It works similar to Controller0
     * \addtogroup Controllers
     */
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
