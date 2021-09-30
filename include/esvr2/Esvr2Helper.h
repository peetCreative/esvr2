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
    //! \brief reparse a RealArray16 to a Ogre Matrix4
    Ogre::Matrix4 RealArray16ToMatrix4(RealArray16 vec);
    //! \brief reparse a Ogre Matrix4 to a RealArray16
    RealArray16 Matrix4ToRealArray16(Ogre::Matrix4 m);
    //! \brief reparse a RealArray3 to a Ogre Vector 3
    Ogre::Vector3 RealArray3ToVector3(RealArray3 vec);
    //! \brief reparse a Ogre Vector3 to a RealArray3
    RealArray3 Vector3ToRealArray3(Ogre::Vector3 vec);
    //! \brief reparse a RealArray4 to a Ogre Quaternion
    Ogre::Quaternion RealArray4ToQuaternion(RealArray4 vec);
    //! \brief reparse a Ogre Quaternion to a RealArray4
    RealArray4 QuaternionToRealArray4(Ogre::Quaternion q);
}

#endif //ESVR2_ESVR2HELPER_H
