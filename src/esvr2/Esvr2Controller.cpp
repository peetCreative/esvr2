//
// Created by peetcreative on 14.12.20.
//
#include "Esvr2Controller.h"

namespace esvr2
{
    Controller::Controller(
            std::shared_ptr<LaparoscopeController> laparoscopeController):
            mLaparoscopeController(laparoscopeController)
    {}
}