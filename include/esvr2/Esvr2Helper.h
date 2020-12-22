//
// Created by peetcreative on 21.12.20.
//
#ifndef ESVR2_ESVR2HELPER_H
#define ESVR2_ESVR2HELPER_H

#include "Esvr2.h"
#include "OgreMatrix4.h"
#include "OgreVector3.h"

namespace esvr2
{
    Ogre::Matrix4 RealVectorToMatrix4(RealVector vec);
    Ogre::Vector3 RealVectorToVector3(RealVector vec);
}

#endif //ESVR2_ESVR2HELPER_H
