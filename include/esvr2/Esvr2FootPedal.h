//
// Created by peetcreative on 17.01.21.
//
#ifdef USE_FOOTPEDAL
#ifndef ESVR2_ESVR2FOOTPEDAL_H
#define ESVR2_ESVR2FOOTPEDAL_H

#include <libserialport.h>

#include <string>

namespace esvr2
{
    enum FootPedalEvent
    {
        FPE_HOVER_PRESS,
        FPE_HOVER_HOLD,
        FPE_HOVER_RELEASE,
        FPE_ACTIVE_PRESS,
        FPE_ACTIVE_HOLD,
        FPE_ACTIVE_RELEASE
    };

    typedef struct sp_port* SerialPortPtr;
    //! \brief Footpedal based on a Arduino connected by a serial console
    //! \deprecated not usefull as we have a USB footpedal
    class FootPedal
    {
    private:
        SerialPortPtr mSerialPort;
        const unsigned int mTimeout = 1000;
        bool mIsReady;
        bool check(enum sp_return result);
        bool isReady();
    public:
        FootPedal(std::string portName);
        ~FootPedal();
        bool pollFootPedalEvent(FootPedalEvent &event);

    };
}

#endif //ESVR2_ESVR2FOOTPEDAL_H
#endif //USE_FOOTPEDAL