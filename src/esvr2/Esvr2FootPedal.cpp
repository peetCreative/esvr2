//
// Created by peetcreative on 17.01.21.
//
#ifdef USE_FOOTPEDAL
#include "Esvr2FootPedal.h"

#include "Esvr2.h"

#include <libserialport.h>
#include <string>

namespace esvr2 {
    FootPedal::FootPedal(std::string portName) {

        /* Open and configure the port. */
        sp_get_port_by_name(portName.c_str(), &mSerialPort);

        check(sp_open(mSerialPort, SP_MODE_READ_WRITE));
        check(sp_set_baudrate(mSerialPort, 9600));
        check(sp_set_bits(mSerialPort, 8));
        check(sp_set_parity(mSerialPort, SP_PARITY_NONE));
        check(sp_set_stopbits(mSerialPort, 1));
        check(sp_set_flowcontrol(mSerialPort, SP_FLOWCONTROL_NONE));
        char buf;
        mIsReady = sp_blocking_read(mSerialPort, &buf, 1, 1000) == SP_OK;
    }

    bool FootPedal::pollFootPedalEvent(FootPedalEvent &event)
    {
        if(!mIsReady)
        {
            return false;
        }
        char buf;
        /* Check whether we received the number of bytes we wanted. */
        if (!check(sp_blocking_read(mSerialPort, &buf, 1, 1)))
            return false;

        switch (buf)
        {
            case '0':
                event = FPE_HOVER_PRESS;
                break;
            case '1':
                event = FPE_HOVER_RELEASE;
                break;
            case '2':
                event = FPE_ACTIVE_PRESS;
                break;
            case '3':
                event = FPE_ACTIVE_RELEASE;
                break;
            default:
                return false;
        }
        return true;
    }

    /* Helper function for error handling. */
    bool FootPedal::check(enum sp_return result)
    {
        if (result == 1)
            return true;

        /* For this example we'll just exit on any error by calling abort(). */
        char *error_message;

        switch (result) {
        case SP_ERR_ARG:
                LOG << "Error: Invalid argument." << LOGEND;
                break;
        case SP_ERR_FAIL:
                error_message = sp_last_error_message();
                LOG << "Error: Failed: " <<  error_message << LOGEND;
                sp_free_error_message(error_message);
                break;
        case SP_ERR_SUPP:
                LOG << "Error: Not supported." << LOGEND;
                break;
        case SP_ERR_MEM:
                LOG << "Error: Couldn't allocate memory." << LOGEND;
                break;
        default:
            break;
        }
        return false;

    }

}
#endif //USE_FOOTPEDAL