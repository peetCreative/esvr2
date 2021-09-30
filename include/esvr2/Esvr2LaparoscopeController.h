//
// Created by peetcreative on 27.11.20.
//

#ifndef ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
#define ESVR2_ESVR2LAPAROSCOPECONTROLLER_H

#include "Esvr2Component.h"
#include "PivotControlMessages.h"
#include <memory>

using namespace pivot_control_messages;

namespace esvr2
{
    //! \brief Superclass wrapping the PivotController together as a Component
    class LaparoscopeController :
            virtual public Component,
            virtual public PivotController {
    public:
        bool isReady() {return PivotController::isReady();};
    };
    typedef std::shared_ptr<LaparoscopeController> LaparoscopeControllerPtr;
}

#endif //ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
