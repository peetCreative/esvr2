#include "Esvr2ParseYml.h"

#include "Esvr2.h"
#include "Esvr2InteractiveElement2DDef.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cassert>
#include <string>

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

    template<typename T>
    void operator >> (const YAML::Node& node, T& i)
    {
      i = node.as<T>();
    }

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

    bool readStereoCameraConfigNodeIntern(
            YAML::Node doc, CameraConfig& cam_info)
    {
        doc[CAM_YML_NAME] >> cam_info.eye_str;
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

    bool readCalibrationYmlIntern(
            std::istream& in, std::string& camera_name, CameraConfig& cam_info)
    {
        try {
            YAML::Node doc = YAML::Load(in);

            if (doc[CAM_YML_NAME])
                doc[CAM_YML_NAME] >> camera_name;
            else
                camera_name = "unknown";

            readStereoCameraConfigNodeIntern(doc, cam_info);

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

    bool readSequence(YAML::Node node, RealVector *vec, int length)
    {
        if (!node.IsSequence())
            return false;
        int i = 0;
        for (YAML::iterator it = node.begin(); it != node.end(); ++it)
        {
            if (i >= length)
                return false;
            vec->at(i++) = it->as<float>();
        }
        if (i != length)
            return false;
        return true;
    }

    bool readColorYmlIntern(
            YAML::Node node, std::vector<float> &colorVal,
            ColorDefList &colorDefList)
    {
        ColorDef colorDef;
        if (node.Type() == YAML::NodeType::Map)
        {
            if (YAML::Node param = node["r"])
                colorDef.color[INDEX_RED] = param.as<float>();
            if (YAML::Node param = node["g"])
                colorDef.color[INDEX_GREEN] = param.as<float>();
            if (YAML::Node param = node["b"])
                colorDef.color[INDEX_BLUE] = param.as<float>();
            if (YAML::Node param = node["a"])
                colorDef.color[INDEX_ALPHA] = param.as<float>();
        }
        else if(node.Type() == YAML::NodeType::Sequence)
        {
            if (node.size() == 4)
            {
                size_t i = 0;
                for (YAML::iterator it = node.begin(); it != node.end(); it++)
                {
                    if (!(*it))
                        return false;
                    colorDef.color[i++] = it->as<float>();
                }
            }
        }
        else if(node.Type() == YAML::NodeType::Scalar)
        {
            std::string str = node.as<std::string>();
            if (str.rfind("Color", 0))
            {
                colorDef.name = str;
            }
            else if(str.rfind("Hex", 0))
            {
//                TODO: create color
            }
            else if(str == "None")
            {
                colorVal = {0, 0, 0, 0};
                return true;
            }
        }
//        auto it = find_if(
//                colorDefList.begin(),
//                colorDefList.end(),
//                [&colorDef](const ColorDef& obj)
//                {return obj.name == colorDef.name ||
//                        (obj.color[INDEX_RED] == colorDef.color[INDEX_RED] &&
//                                obj.color[INDEX_GREEN] == colorDef.color[INDEX_GREEN] &&
//                                obj.color[INDEX_BLUE] == colorDef.color[INDEX_BLUE] &&
//                                obj.color[INDEX_ALPHA] == colorDef.color[INDEX_ALPHA]);});
        std::string nameInList = "";
        for (auto it = colorDefList.begin(); it != colorDefList.end(); it++)
        {
            if (it->name == colorDef.name ||
                (it->color[INDEX_RED] == colorDef.color[INDEX_RED] &&
                 it->color[INDEX_GREEN] == colorDef.color[INDEX_GREEN] &&
                 it->color[INDEX_BLUE] == colorDef.color[INDEX_BLUE] &&
                 it->color[INDEX_ALPHA] == colorDef.color[INDEX_ALPHA]))
            {
                nameInList = it->name;
                break;
            }
        }
        if (nameInList == "")
        {
            std::stringstream ss;
            ss << "ColorHex" << std::hex;
            for (int i = 0; i < 4; i++)
            {
                int colorVal = static_cast<int>(255 * colorDef.color[i]);
                ss << colorVal;
            }
            colorDef.name = ss.str();
            colorDefList.push_back(colorDef);
        }
        colorVal = colorDef.color;
        return true;
    }

    bool readInteractiveElementConfigYmlIntern(
            YAML::Node node, InteractiveElementConfig& config)
    {
        if (YAML::Node param = node["width"])
            config.width = param.as<float>();
        if (YAML::Node param = node["height"])
            config.height = param.as<float>();
        if (YAML::Node param = node["bg_img"])
            config.bgImg = param.as<std::string>();
        if (YAML::Node param = node["bg_color"])
        {
            readColorYmlIntern(param, config.bgColor, config.colorDefList);
        }
        if (YAML::Node param = node["definitions"])
        {
            for (YAML::iterator it = param.begin(); it != param.end(); it++)
            {
                YAML::Node elem = *it;
                if (!elem)
                {
                    LOG << "could not read InteractiveElement" << LOGEND;
                    continue;
                }
                InteractiveElement2DDef def;
                if (YAML::Node param = elem["id"])
                    def.id = param.as<std::string>();
                if (YAML::Node param = elem["uv_x"])
                {
                    float uvx = param.as<float>();
                    def.uvX = uvx >= 0 ? uvx : 1.0 + uvx;
                }
                if (YAML::Node param = elem["uv_y"])
                {
                    float uvy = param.as<float>();
                    def.uvY = uvy >= 0 ? uvy : 1.0 + uvy;

                }
                if (YAML::Node param = elem["uv_size_x"])
                {
                    std::string str = param.as<std::string>();
                    if ( str == "rest")
                        def.uvSizeX = 1.0 - def.uvX;
                    else
                        def.uvSizeX = param.as<float>();
                }
                if (YAML::Node param = elem["uv_size_y"])
                {
                    std::string str = param.as<std::string>();
                    if ( str == "rest")
                        def.uvSizeY = 1.0 - def.uvY;
                    else
                        def.uvSizeY = param.as<float>();
                }
                if (YAML::Node param = elem["text"])
                    def.text = param.as<std::string>();
                if (YAML::Node param = elem["font"])
                    def.font = param.as<std::string>();
                if (YAML::Node param = elem["font_size"])
                    def.fontSize = param.as<float>();
                if (YAML::Node param = elem["font_size_rel"])
                    def.fontSize = 0.05f * param.as<float>();
                if (YAML::Node param = elem["fit_font_size"])
                    def.fitFontSize = param.as<bool>();
                if (YAML::Node param = elem["font_color"])
                {
                    readColorYmlIntern(param, def.fontColor, config.colorDefList);
                }
                if (YAML::Node param = elem["always_visible"])
                    def.alwaysVisible = param.as<bool>();
                if (YAML::Node param = elem["bg_color"])
                {
                    readColorYmlIntern(param, def.bgColor, config.colorDefList);
                }
                if (YAML::Node param = elem["bg_color_hover"])
                {
                    readColorYmlIntern(param, def.bgHoverColor, config.colorDefList);
                }
                if (YAML::Node param = elem["bg_color_active"])
                {
                    readColorYmlIntern(param, def.bgActiveColor, config.colorDefList);
                }
                if (YAML::Node param = elem["visible_on_active"])
                    def.visibleOnActive = param.as<bool>();
                if (YAML::Node param = elem["hide_other_on_active"])
                    def.hideOtherOnActive = param.as<bool>();
                if(!config.findByName(def.id))
                {
                    InteractiveElement2DDefPtr defPtr =
                            std::make_shared<InteractiveElement2DDef>(def);
                    config.defList.push_back(defPtr);
                }
            }
            return true;
        }
        return true;
    }

    bool readInteractiveElementConfigYml(
            const std::string& file_name,
            InteractiveElementConfig& config)
    {
        LOG << "file_name:" << file_name << LOGEND;
        std::ifstream fin(file_name.c_str());
        if (!fin.good()) {
            LOG << "Unable to open InteractiveElement2DDef configfile: ["
                << file_name << "]" << LOGEND;
            return false;
        }
        YAML::Node node = YAML::Load(fin);
        bool success = false;
        //TODO: rename to sth consistent
        if(YAML::Node bordNode = node["interactive_bord"])
        {
            success = readInteractiveElementConfigYmlIntern(bordNode, config);
            if (!success)
                LOG << "Failed to parse InteractiveElement2DDef configfile from file ["
                    << file_name << " %s]" << LOGEND;

        }
        return success;
    }


    bool readHmdConfigYmlIntern(YAML::Node doc, HmdConfig* hmdConfig)
    {
        bool succ = true;
        if (YAML::Node widthNode = doc["width"])
            hmdConfig->width = widthNode.as<int>();
        else return false;
        if (YAML::Node heightNode = doc["height"])
            hmdConfig->height = heightNode.as<int>();
        else return false;
        std::vector<std::string> eyeStr = {"left", "right"};
        for (size_t eye = 0; eye < 2; eye++ )
        {
            if (YAML::Node eyeNode = doc[eyeStr[eye]])
            {
                YAML::Node e2hNode = eyeNode["eye_to_head"];
                YAML::Node pmNode = eyeNode["projection_matrix"];
                YAML::Node tanNode = eyeNode["tan"];
                succ = succ && e2hNode && readSequence(e2hNode, hmdConfig->eyeToHeadPtr[eye], 3);
                succ = succ && pmNode && readSequence(pmNode, hmdConfig->projectionMatrixPtr[eye], 16);
                succ = succ && tanNode && readSequence(tanNode, hmdConfig->tanPtr[eye], 4);
            }
        }
        return succ && hmdConfig->valid();
    }

    bool readConfigYmlIntern(std::istream& in,
                       std::shared_ptr<Esvr2Config> config,
                       std::shared_ptr<Esvr2VideoInputConfig> videoInputConfig = nullptr)
    {
        YAML::Node doc = YAML::Load(in);
        if (YAML::Node param = doc["show_video"])
            config->showVideo = param.as<bool>();
        if (YAML::Node param = doc["show_ogre_dialog"])
            config->showOgreDialog = param.as<bool>();
        if (YAML::Node param = doc["multithreading"])
            config->multithreading = param.as<bool>();
        if (YAML::Node param = doc["is_stereo"])
            config->isStereo = param.as<bool>();
        if (YAML::Node param = doc["debug_screen"])
            config->debugScreen = param.as<bool>();
        if (YAML::Node param = doc["screen"])
            config->screen = param.as<int>();
        if (YAML::Node param = doc["head_hight"])
            config->headHight = param.as<double>();
        if (YAML::Node param = doc["controller_type"])
            config->controllerType =
                getControllerType(param.as<std::string>());
        if (YAML::Node param = doc["controller_opt01_delay"])
            config->ctlDelay = param.as<int>();
        if (YAML::Node param = doc["controller_step_trans_z"])
            config->ctlStepTransZ = param.as<float>();
        if (YAML::Node param = doc["controller_step_yaw"])
            config->ctlStepYaw = param.as<float>();
        if (YAML::Node param = doc["controller_step_pitch"])
            config->ctlStepPitch = param.as<float>();
        if (YAML::Node param = doc["controller_step_roll"])
            config->ctlStepRoll = param.as<float>();
        if (YAML::Node param = doc["controller_opt0_threshold_trans_z"])
            config->ctlOpt0ThresholdTransZ = param.as<float>();
        if (YAML::Node param = doc["controller_opt0_threshold_yaw_deg"])
            config->ctlOpt0ThresholdYawDeg = param.as<float>();
        if (YAML::Node param = doc["controller_opt0_threshold_pitch_deg"])
            config->ctlOpt0ThresholdPitchDeg = param.as<float>();
        if (YAML::Node param = doc["controller_opt0_threshold_roll_deg"])
            config->ctlOpt0ThresholdRollDeg = param.as<float>();
        if (YAML::Node param = doc["workspace_type"])
            config->workspaceType =
                getWorkspaceType(param.as<std::string>());
        if (YAML::Node param = doc["render_video_target"])
            config->videoRenderTarget =
                getRenderVideoTarget(param.as<std::string>());
        if (YAML::Node param = doc["resource_path"])
            config->resourcePath = param.as<std::string>();
        if (YAML::Node param = doc["log_folder"])
            config->logFolder = param.as<std::string>();
        if (YAML::Node param = doc["serial_port"])
            config->serialPort = param.as<std::string>();
        if (YAML::Node param = doc["center_projection_plane"])
            config->centerProjectionPlane = param.as<bool>();

        YAML::Node doc_video = doc["video"];
        if (doc_video && videoInputConfig)
        {
            if (YAML::Node param = doc_video["is_stereo"])
            {
                config->isStereo = param.as<bool>();
                videoInputConfig->isStereo = param.as<bool>();
            }
            if (YAML::Node param = doc_video["input_type"])
                videoInputConfig->inputType =
                        getInputType(param.as<std::string>());
            if (YAML::Node param = doc_video["video_input_type"])
                videoInputConfig->videoInputType =
                        getVideoInputType(param.as<std::string>());
            if (YAML::Node param = doc_video["path"])
                videoInputConfig->path = param.as<std::string>();
            if (YAML::Node param = doc_video["distortion"])
                videoInputConfig->distortion =
                        getDistortionType(param.as<std::string>());
            if (YAML::Node param = doc_video["wait_for_camera_config"])
                videoInputConfig->wait4CameraConfig =
                        param.as<bool>();
        }
        bool succ = true;
        YAML::Node param = doc["stereo_camera_config_left"];
        if (param && videoInputConfig)
            succ = succ && readStereoCameraConfigNodeIntern(
                    param,
                    videoInputConfig->stereoCameraConfig.leftCameraConfig);
        param = doc["stereo_camera_config_right"];
        if (param && videoInputConfig)
            succ = succ && readStereoCameraConfigNodeIntern(
                    param,
                    videoInputConfig->stereoCameraConfig.rightCameraConfig);
        if (YAML::Node param = doc["hmd_info"])
            succ = succ && readHmdConfigYmlIntern(param, &(config->hmdConfig));
        return succ;
    }

    bool readConfigYml(const std::string& file_name,
                       std::shared_ptr<Esvr2Config> config,
                       std::shared_ptr<Esvr2VideoInputConfig> videoInputConfig)
    {
        LOG << "file_name:" << file_name << LOGEND;
        std::ifstream fin(file_name.c_str());
        if (!fin.good()) {
            LOG << "Unable to open Config file [" << file_name << "]" << LOGEND;
            return false;
        }
        bool success = readConfigYmlIntern(fin, config, videoInputConfig);
        if (!success)
            LOG << "Failed to parse camera calibration from file [" << file_name << " %s]" << LOGEND;
        return success;
    }
} //namespace esvr2
