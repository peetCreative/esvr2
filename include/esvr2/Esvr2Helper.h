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
    Ogre::Matrix4 RealArray16ToMatrix4(RealArray16 vec);
    RealArray16 Matrix4ToRealArray16(Ogre::Matrix4 m);
    Ogre::Vector3 RealArray3ToVector3(RealArray3 vec);
    RealArray3 Vector3ToRealArray3(Ogre::Vector3 vec);
    Ogre::Quaternion RealArray4ToQuaternion(RealArray4 vec);
    RealArray4 QuaternionToRealArray4(Ogre::Quaternion q);
}

#endif //ESVR2_ESVR2HELPER_H
