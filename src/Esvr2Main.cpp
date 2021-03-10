#include "Esvr2.h"
#include "Esvr2ParseYml.h"
#include "Esvr2BlackMagicVideoLoader.h"
#include "Esvr2LowLatencyVideoLoader.h"
#include "Esvr2OpenCvVideoLoader.h"
#include "Esvr2PoseState.h"
#include "Esvr2TestPose.h"

#include <string>
#include <memory>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{
    size_t config_files_end = 0, config_files_begin = 0;

    bool use_test_pose = false;
    std::shared_ptr<esvr2::Esvr2Config> config =
            std::make_shared<esvr2::Esvr2Config>();
    esvr2::VideoInputConfigPtr videoInputConfig =
            std::make_shared<esvr2::VideoInputConfig>();

//    config->hmdConfig = {
//            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
//            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
//            { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45}},
//            1920, 1080 };

    for (int i = 1; i < argc; i++)
    {
        std::string argument(argv[i]);
        if (argument == "--config" && i+1 < argc )
        {
            i++;
            config_files_begin = i;
            config_files_end = i;
            std::string config_file(argv[i]);
            while( i < argc && config_file.at(0) != '-')
            {
                //we can handle as many config-files as possible
                config_files_end++;
                config_file = std::string(argv[i++]);
            }
            if( i >= argc )
                break;
        }
        if ( argument == "--input-type" && i+1 < argc )
        {
            videoInputConfig->inputType = esvr2::getInputType(argv[i+1]);
        }
        if ( argument == "--video-path" && i+1 < argc )
        {
            videoInputConfig->inputType = esvr2::IT_VIDEO_OPENCV;
            videoInputConfig->path = std::string(argv[i+1]);
        }
        if ( argument == "--show-ogre-dialog" )
            config->showOgreDialog = true;
        if ( argument == "--multithreading" )
            config->multithreading = true;
        if ( argument == "--test-pose" )
            use_test_pose = true;
    }

    for (size_t i = config_files_begin; i < config_files_end; i++ )
    {
        std::string config_file;
        std::string path_str(argv[i]);
        if ( path_str.back() == '/' )
        {
            for( auto& p: fs::directory_iterator(path_str) )
            {
                config_file = p.path();
            }
        }
        else
        {
            config_file = path_str;
        }
        esvr2::readConfigYml( config_file, config, videoInputConfig );
    }

    //create and initialize the videoLoader
    std::shared_ptr<esvr2::VideoLoader> videoLoader = nullptr;
    switch(videoInputConfig->inputType) {
        case esvr2::IT_VIDEO_LOW_LATENCY:
#ifdef USE_LOWLATENCYVIDEOLOADER
            videoLoader = std::make_shared<esvr2::LowLatencyVideoLoader>(
                    videoInputConfig, false);
#endif
            break;
        case esvr2::IT_VIDEO_OPENCV:
            videoLoader = std::make_shared<esvr2::OpenCvVideoLoader>(
                    videoInputConfig);
            break;
        case esvr2::IT_VIDEO_BLACKMAGIC:
#ifdef USE_BLACKMAGICCAMERA
        videoLoader = std::make_shared<esvr2::BlackMagicVideoLoader>(
                    videoInputConfig);
#endif
            break;
            case esvr2::IT_NONE:
            LOG << "no input: shutdown" << LOGEND;
            return 1;
    }

    //TODO: create poseState
    std::shared_ptr<esvr2::PoseState> poseState;
    poseState = nullptr;
    if(use_test_pose)
    {
        poseState = std::make_shared<esvr2::TestPose>();
    }

    bool validCameraConfig = /*cameraConfig->leftToRight != 0 &&*/
            videoInputConfig->stereoCameraConfig.cfg[LEFT]->valid() &&
            ( !videoInputConfig->isStereo ||
            ( videoInputConfig->isStereo &&
                videoInputConfig->stereoCameraConfig.cfg[LEFT]->valid()));
    if ( !validCameraConfig )
    {
        LOG << "no valid cameraConfig quit" << LOGEND;
    }


    //create esvr2
    esvr2::Esvr2 *esvr2 = new esvr2::Esvr2(
            config, videoLoader, nullptr, poseState );
    // run esvr2
    return esvr2->run();
}
