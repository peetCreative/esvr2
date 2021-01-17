//
// Created by peetcreative on 13.12.20.
//

#ifndef ESVR2_ESVR2OPT0CONTROLLER_H
#define ESVR2_ESVR2OPT0CONTROLLER_H
#include "Esvr2Controller.h"

#include "SDL.h"
namespace esvr2
{
    class Opt0Controller : public Controller
    {
    private:
        bool mMoveMode;
    public:
        Opt0Controller(
                std::shared_ptr<LaparoscopeController> laparoscopeController,
                GameState *gameState);
        bool keyPressed( const SDL_KeyboardEvent &arg ) override;
        bool keyReleased( const SDL_KeyboardEvent &arg ) override;
        void headPoseUpdated() override;
    };
}


#endif //ESVR2_ESVR2OPT0CONTROLLER_H
