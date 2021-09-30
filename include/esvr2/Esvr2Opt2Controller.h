//
// Created by peetcreative on 11.01.21.
//

#ifndef ESVR2_OPT2CONTROLLER_H
#define ESVR2_OPT2CONTROLLER_H
#include "Esvr2Controller.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2GameState.h"

#include "PivotControlMessages.h"
using namespace pivot_control_messages;

namespace esvr2
{
    //! \brief Laparoscope Controller, which directly follows the head-motions when activated
    /*! \remarks
     * When holding the left footpedal (activating non-visible interactive element),
     * the controller tracks the rotational angles/ and forward backward distance
     * of the head movements and translates them into pitch/yaw/trans_z movements
     *
     * roll movements are supposed to be compensated
     * as the surgeon assumes the image to be horizontially aligned
     *
     * \addtogroup Controllers
     */
    class Opt2Controller : public Controller
    {
    private:
        Ogre::Quaternion mStartOrientation = Ogre::Quaternion::ZERO;
        Ogre::Vector3 mStartPosition = Ogre::Vector3::ZERO;
        DOFPose mStartPose;
        Ogre::Real mTransZFact = 2.0;
        Ogre::Real mCamereaTilt;
        Ogre::Real mFocusDistance;
        bool mBlocked = false;
        bool mEnableTransPitch = false;
        bool mFollowMovements = false;
    public:
        Opt2Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState,
                Ogre::Real transZFact,
                Ogre::Real cameraTilt,
                Ogre::Real focusDistance);

        void startMoving();
        void hold(Ogre::uint64 time);
        std::string getControllerMenuId() override;
    };
}

#endif //ESVR2_OPT2CONTROLLER_H
