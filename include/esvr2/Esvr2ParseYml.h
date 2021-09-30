#ifndef _Esvr2_Parse_Yml_H
#define _Esvr2_Parse_Yml_H
/** \file Esvr2ParseYml.h
 * File containing declarations for methods parsing diverse Yaml-Files.
 * \author Peter Klausing
 */

#include "Esvr2.h"
#include "Esvr2InteractiveElement2DDef.h"
#include <string>

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
/**
 * \brief Read interactive element config from YAML file.
 *
 * \param file_name File to read
 * \param[out] config reference config to be written to
 */
bool readInteractiveElementConfigYml(
        const std::string& file_name,
        InteractiveElementConfig& config);

/**
 * \brief Read config from YAML file.
 *
 * \param file_name File to read
 * \param[out] config reference to Esvr2Config -Object to be written to
 * \param[out] videoInputConfig reference to VideoInputConfig -Object to be written to
 */
bool readConfigYml(const std::string& file_name,
                   std::shared_ptr<Esvr2Config> config,
                   VideoInputConfigPtr videoInputConfig = nullptr);

/**
 * \brief Read cached information from YAML file.
 *
 * reads cached orientation and distance of the projection plane
 *
 * \param file_name Cache file to read
 * \param[out] config reference to Esvr2Config -Object to be written to
 */
bool readCacheYml(std::shared_ptr<Esvr2Config> config);
/**
 * \brief write orientation and distance to cahce in YAML format.
 *
 * reads cached orientation and distance of the projection plane
 * and writes it to a yaml file
 *
 * \param[out] config reference to Esvr2Config -Object the data is read from
 */

bool writeCacheYml(std::shared_ptr<Esvr2Config> config);

} //namespace esvr2

#endif
