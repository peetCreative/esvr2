/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "ParseYml.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ctime>
#include <cassert>
#include <cstring>
#include <ros/console.h>

namespace esvr2 {

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

struct SimpleMatrix
{
  int rows;
  int cols;
  float* data;

  SimpleMatrix(int rows, int cols, float* data)
    : rows(rows), cols(cols), data(data)
  {}
};

YAML::Emitter& operator << (YAML::Emitter& out, const SimpleMatrix& m)
{
  out << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << m.rows;
  out << YAML::Key << "cols" << YAML::Value << m.cols;
  //out << YAML::Key << "dt"   << YAML::Value << "d"; // OpenCV data type specifier
  out << YAML::Key << "data" << YAML::Value;
  out << YAML::Flow;
  out << YAML::BeginSeq;
  for (int i = 0; i < m.rows*m.cols; ++i)
    out << m.data[i];
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

void operator >> (const YAML::Node& node, SimpleMatrix& m)
{
  int rows, cols;
  node["rows"] >> rows;
  assert(rows == m.rows);
  node["cols"] >> cols;
  assert(cols == m.cols);
  const YAML::Node& data = node["data"];
  for (int i = 0; i < rows*cols; ++i)
    data[i] >> m.data[i];
}

    bool readCalibrationYmlIntern(std::istream& in, std::string& camera_name, CameraConfig& cam_info)
    {
        try {
        #ifdef HAVE_NEW_YAMLCPP
            YAML::Node doc = YAML::Load(in);

            if (doc[CAM_YML_NAME])
            doc[CAM_YML_NAME] >> camera_name;
            else
            camera_name = "unknown";
        #else
            YAML::Parser parser(in);
            if (!parser) {
            ROS_ERROR("Unable to create YAML parser for camera calibration");
            return false;
            }
            YAML::Node doc;
            parser.GetNextDocument(doc);

            if (const YAML::Node* name_node = doc.FindValue(CAM_YML_NAME))
            *name_node >> camera_name;
            else
            camera_name = "unknown";
        #endif

            doc[WIDTH_YML_NAME] >> cam_info.width;
            doc[HEIGHT_YML_NAME] >> cam_info.height;

            // Read in fixed-size matrices
            SimpleMatrix K_(3, 3, &cam_info.K[0]);
            doc[K_YML_NAME] >> K_;
            SimpleMatrix R_(3, 3, &cam_info.R[0]);
            doc[R_YML_NAME] >> R_;
            SimpleMatrix P_(3, 4, &cam_info.P[0]);
            doc[P_YML_NAME] >> P_;

            const YAML::Node& D_node = doc[D_YML_NAME];
            int D_rows, D_cols;
            D_node["rows"] >> D_rows;
            D_node["cols"] >> D_cols;
            const YAML::Node& D_data = D_node["data"];
            cam_info.D.resize(D_rows*D_cols);
            for (int i = 0; i < D_rows*D_cols; ++i)
            D_data[i] >> cam_info.D[i];
            return true;
        }
        catch (YAML::Exception& e) {
            LOG << " could not read yaml correctly" << LOGEND;
            return false;
        }
    }

    bool readCalibrationYml(const std::string& file_name,
        std::string& camera_name, CameraConfig& cam_info)
    {
    std::ifstream fin(file_name.c_str());
    if (!fin.good()) {
        LOG << "Unable to open camera calibration file [" << file_name << "]" << LOGEND;
        return false;
    }
    bool success = readCalibrationYmlIntern(fin, camera_name, cam_info);
    if (!success)
        LOG << "Failed to parse camera calibration from file [" << file_name << " %s]" << LOGEND;
    return success;
    }

} //namespace esvr2
