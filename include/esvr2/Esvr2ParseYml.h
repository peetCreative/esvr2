#ifndef _Esvr2_Parse_Yml_H
#define _Esvr2_Parse_Yml_H

#include "Esvr2.h"

#include <string>
#include <istream>
#include <ostream>

namespace esvr2 {

/**
 * \brief Read calibration parameters from a YAML file.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        CameraConfig& cam_info);
bool readConfigYml(const std::string& file_name,
                   std::shared_ptr<Esvr2Config> config,
                   std::shared_ptr<Esvr2VideoInputConfig> videoInputConfig = nullptr);

} //namespace esvr2

#endif
