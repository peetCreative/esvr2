//
// Created by peetcreative on 13.12.20.
//
#include "Esvr2KeyboardController.h"
#include "Esvr2Controller.h"

namespace esvr2
{
    KeyboardController::KeyboardController(
            std::shared_ptr<LaparoscopeController> laparoscopeController):
            Controller(laparoscopeController)
    {}
}