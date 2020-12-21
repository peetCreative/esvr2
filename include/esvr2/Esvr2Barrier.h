//
// Created by peetcreative on 16.12.20.
//

#ifndef ESVR2_ESVR2BARRIER_H
#define ESVR2_ESVR2BARRIER_H

#include "Threading/OgreBarrier.h"

namespace esvr2
{
    class Barrier
    {
    private:
        Ogre::Barrier mBarrier;
    public:
        Barrier();
        void sync();
    };
}

#endif //ESVR2_ESVR2BARRIER_H
