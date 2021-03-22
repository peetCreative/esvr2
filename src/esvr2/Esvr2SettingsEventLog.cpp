//
// Created by peetcreative on 13.02.21.
//
#include "Esvr2SettingsEventLog.h"

#include "Ogre.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <filesystem>

namespace esvr2
{
    bool writeSettingsEventLog(
            std::string folderPath,
            std::string prefix,
            SettingsEventLogList logList,
            Ogre::uint64 msTime)
    {
        auto now = std::chrono::system_clock::now();
        time_t nowTm = std::chrono::system_clock::to_time_t(now);
        std::stringstream filePath;
        filePath << folderPath
                 << "/esvr2SettingsLog_"
                 << prefix << "_"
                 << std::put_time(std::localtime(&nowTm), "%FT%H:%M:%S")
                 << ".yml";
        YAML::Node rootNode;
        rootNode["Duration"] = msTime;
        YAML::Node logsNode;
        for(auto logIt = logList.begin(); logIt != logList.end(); logIt++ )
        {
            YAML::Node logNode;
            logNode["time"] = logIt->time;
            logNode["event"] = logIt->event;
            logNode["projection_plane_distanceRaw"] = logIt->projectionPlaneDistanceRaw;
            logNode["projection_plane_distanceRect"] = logIt->projectionPlaneDistanceRect;
            std::stringstream orientationss;
            orientationss << logIt->projectionPlaneOrientation;
            logNode["projection_plane_orientation"] = orientationss.str();
            std::stringstream headposess;
            headposess << logIt->headPosition;
            logNode["head_position"] = headposess.str();
            logNode["eye_dist"] = logIt->eyeDist;
            logNode["distortion"] = logIt->distortion == DIST_RAW? "RAW" : "UNDIST_RECT";
            logsNode.push_back(logNode);
        }
        rootNode["Logs"] = logsNode;
        std::ofstream fout;
        fout.open(filePath.str());
        fout << rootNode;
        fout.close();
        return true;
    }

}