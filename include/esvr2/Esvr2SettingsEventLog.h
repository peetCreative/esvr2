//
// Created by peetcreative on 12.02.21.
//

#ifndef ESVR2_SETTINGSEVENTLOG_H
#define ESVR2_SETTINGSEVENTLOG_H

#include "Ogre.h"
#include "OgreQuaternion.h"
#include "OgreVector3.h"

#include "Esvr2.h"

namespace esvr2
{
    struct SettingsEventLog
    {
        Ogre::uint64 time = 0;
        Ogre::String event = "";
        Ogre::Real projectionPlaneDistanceRaw = 0;
        Ogre::Real projectionPlaneDistanceRect  = 0;
        Ogre::Quaternion projectionPlaneOrientation = Ogre::Quaternion::ZERO;
        Ogre::Vector3 headPosition = Ogre::Vector3::ZERO;
        Ogre::Real eyeDist = 0;
        Distortion distortion = DIST_RAW;
    };
    typedef std::vector<SettingsEventLog> SettingsEventLogList;

    bool writeSettingsEventLog(
            std::string folderPath,
            std::string prefix,
            SettingsEventLogList logList,
            Ogre::uint64 msTime);

}

#endif //ESVR2_ESVR2SETTINGSEVENTLOG_H
