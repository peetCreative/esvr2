//
// Created by peetcreative on 13.12.20.
//

#ifndef ESVR2_ESVR2KEYBOARDCONTROLLER_H
#define ESVR2_ESVR2KEYBOARDCONTROLLER_H

#include "Esvr2Controller.h"

namespace esvr2
{
    class KeyboardController : public Controller
    {
    private:
    public:
        KeyboardController(std::shared_ptr<LaparoscopeController> laparoscopeController);
    };
}


#endif //ESVR2_ESVR2KEYBOARDCONTROLLER_H
