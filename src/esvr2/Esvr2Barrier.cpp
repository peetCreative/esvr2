//
// Created by peetcreative on 16.12.20.
//
#include "Esvr2Barrier.h"

namespace esvr2
{
    Barrier::Barrier():
    mBarrier(2)
    {}

    void Barrier::sync()
    {
        mBarrier.sync();
    }
}