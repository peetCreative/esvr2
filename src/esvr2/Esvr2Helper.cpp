//
// Created by peetcreative on 21.12.20.
//
#include "Esvr2Helper.h"

namespace esvr2
{
    Ogre::Matrix4 RealArray16ToMatrix4(RealArray16 vec)
    {
        return Ogre::Matrix4(vec[0], vec[1], vec[2], vec[3],
                             vec[4], vec[5], vec[6], vec[7],
                             vec[8], vec[9], vec[10],vec[11],
                             vec[12],vec[13],vec[14],vec[15]);
    }

    RealArray16 Matrix4ToRealArray16(Ogre::Matrix4 m)
    {
        return {m[0][0], m[0][1], m[0][2], m[0][3],
            m[1][0], m[1][1], m[1][2], m[1][3],
            m[2][0], m[2][1], m[2][2], m[2][3],
            m[3][0], m[3][1], m[3][2], m[3][3]};
    }

    Ogre::Vector3 RealArray3ToVector3(RealArray3 vec)
    {
        return Ogre::Vector3(vec[0], vec[1], vec[2]);
    }

    RealArray3 Vector3ToRealArray3(Ogre::Vector3 vec)
    {
        return {vec[0], vec[1], vec[2]};
    }

    Ogre::Quaternion RealArray4ToQuaternion(RealArray4 vec)
    {
        return Ogre::Quaternion(vec[0], vec[1], vec[2], vec[3]);
    }

    RealArray4 QuaternionToRealArray4(Ogre::Quaternion q)
    {
        return {q[0], q[1], q[2], q[3]};
    }
}
