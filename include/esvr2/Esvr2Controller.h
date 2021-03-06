//
// Created by peetcreative on 14.12.20.
//

#ifndef ESVR2_ESVR2CONTROLLER_H
#define ESVR2_ESVR2CONTROLLER_H

#include "Esvr2InteractiveElement.h"
#include "Ogre.h"

#include <boost/function.hpp>
#include <memory>
#include <vector>
#include "SDL.h"

namespace esvr2
{
    class LaparoscopeController;
    class GameState;
    /* This class is to be overwritten to implement different
     * modalities to control the Laparoscope.
     * It can use the OpenVR transforms, add things to the VR scene
     * and to the  Controll UI.
     */
    class Controller : public InteractiveElement
    {
    protected:
        std::shared_ptr<LaparoscopeController> mLaparoscopeController;
        GameState *mGameState;
    public:
        Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);
        virtual void headPoseUpdated() {};
        virtual std::string getControllerMenuId() = 0;
        virtual bool isActiveOnPress()
        {
            return mTogglePressCallback || mHoldCallback || mToggleReleaseCallback;
        };
        void stopMotion();
    };
    typedef std::shared_ptr<Controller> ControllerPtr;
    typedef std::vector<ControllerPtr> ControllerPtrList;
}

#endif //ESVR2_ESVR2CONTROLLER_H
