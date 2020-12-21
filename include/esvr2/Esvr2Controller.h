//
// Created by peetcreative on 14.12.20.
//

#ifndef ESVR2_ESVR2CONTROLLER_H
#define ESVR2_ESVR2CONTROLLER_H

#include <memory>

namespace esvr2
{
    class LaparoscopeController;
    /* This class is to be overwritten to implement different
     * modalities to control the Laparoscope.
     * It can use the OpenVR transforms, add things to the VR scene
     * and to the  Controll UI.
     */
    class Controller
    {
    private:
        std::shared_ptr<LaparoscopeController> mLaparoscopeController;
    public:
        Controller(std::shared_ptr<LaparoscopeController> laparoscopeController);
    };
}

#endif //ESVR2_ESVR2CONTROLLER_H
