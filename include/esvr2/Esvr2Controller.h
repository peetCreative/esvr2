//
// Created by peetcreative on 14.12.20.
//

#ifndef ESVR2_ESVR2CONTROLLER_H
#define ESVR2_ESVR2CONTROLLER_H

#include <memory>
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
    class Controller
    {
    protected:
        std::shared_ptr<LaparoscopeController> mLaparoscopeController;
        GameState *mGameState;
    public:
        Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);
        virtual bool keyPressed( const SDL_KeyboardEvent &arg ) {return false;};
        virtual bool keyReleased( const SDL_KeyboardEvent &arg ) {return false;};
        virtual void headPoseUpdated() {};
    };
}

#endif //ESVR2_ESVR2CONTROLLER_H
