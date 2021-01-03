//
// Created by peetcreative on 13.12.20.
//

#ifndef ESVR2_ESVR2KEYBOARDCONTROLLER_H
#define ESVR2_ESVR2KEYBOARDCONTROLLER_H

#include "Esvr2Controller.h"

#include "SDL.h"
namespace esvr2
{
    class KeyboardController : public Controller
    {
    private:
        bool mMoveMode;
    public:
        KeyboardController(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);
        bool keyPressed( const SDL_KeyboardEvent &arg ) override;
        bool keyReleased( const SDL_KeyboardEvent &arg ) override;
        void headPoseUpdated() override;
    };
}


#endif //ESVR2_ESVR2KEYBOARDCONTROLLER_H
