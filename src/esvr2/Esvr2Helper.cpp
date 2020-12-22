//
// Created by peetcreative on 21.12.20.
//
#include "Esvr2Helper.h"

namespace esvr2
{
    Ogre::Matrix4 RealVectorToMatrix4(RealVector vec)
    {
        return Ogre::Matrix4(vec[0], vec[1], vec[2], vec[3],
                             vec[4], vec[5], vec[6], vec[7],
                             vec[8], vec[9], vec[10],vec[11],
                             vec[12],vec[13],vec[14],vec[15]);
    }

    Ogre::Vector3 RealVectorToVector3(RealVector vec)
    {
        return Ogre::Vector3(vec[0], vec[1], vec[2]);
    }
}
