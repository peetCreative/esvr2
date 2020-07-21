#ifndef _Esvr2_StereoRendering_H_
#define _Esvr2_StereoRendering_H_

#include "OgreMatrix4.h"
#include "openvr.h"
#include "opencv2/opencv.hpp"

#define LEFT 0
#define RIGHT 1
#define LOG std::cout
#define LOGEND std::endl

#ifndef RESOURCE_FOLDER
#define RESOURCE_FOLDER "../Data/"
#endif
#ifndef PLUGIN_FOLDER
#define PLUGIN_FOLDER "./"
#endif

namespace esvr2 {
    typedef enum {
        IT_NONE,
        IT_ROS,
        IT_VIDEO_LOW_LATENCY,
        IT_VIDEO_OPENCV,
        IT_VIDEO_BLACKMAGIC
    } InputType;
    typedef enum {
        VRT_TO_BACKGROUND,
        VRT_TO_SQUARE,
        VRT_TO_2D_RECTANGLE
    } VideoRenderTarget;
    typedef enum {
        VIT_NONE,
        VIT_MONO,
        VIT_STEREO_SLICED,
        VIT_STEREO_VERTICAL_SPLIT,
        VIT_STEREO_HORIZONTAL_SPLIT
    } VideoInputType;
    typedef enum {
        RIT_NONE,
        RIT_MONO,
        RIT_STEREO_SLICED,
        RIT_STEREO_SPLIT
    } RosInputType;
    typedef enum {
        DIST_RAW,
        DIST_UNDISTORT,
        DIST_UNDISTORT_RECTIFY
    } Distortion;
    typedef enum {
        ORIENTATION_VERTICAL,
        ORIENTATION_HORIZONTAL
    } Orientation;
    typedef struct {
        std::string path;
        VideoInputType videoInputType;
    } VideoInput;
    typedef enum {
        WS_TWO_CAMERAS_STEREO,
        WS_INSTANCED_STEREO
    } WorkspaceType;
    typedef struct {
        Ogre::Matrix4 eyeToHead[2];
        Ogre::Matrix4 projectionMatrix[2];
        Ogre::Vector4 tan[2];
        Ogre::uint32 width;
        Ogre::uint32 height;
    } HmdConfig;
    typedef struct {
        int width = 0;
        int height = 0;
        std::vector<float> D = { 0, 0, 0, 0, 0 };
        std::vector<float> K = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::vector<float> R = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::vector<float> P = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    } CameraConfig;
    bool valid(CameraConfig cc)
    {
        //TODO: check maybe also the other parameters
        return cc.width != 0 && cc.height != 0;
    };
    typedef struct {
        CameraConfig cfg[2];
        float leftToRight = 0;
    } StereoCameraConfig;
    //similar implementation to cv::Mat*
    typedef struct {
        //-1 invalid frame
        int frameId = -1;
        size_t length = 0;
        size_t rows = 0;
        size_t cols = 0;
        size_t depth = 0;
        Ogre::uint8 *data = nullptr;
    } ImageData;
    bool valid(ImageData id)
    {
        return id.id != 0 && id.length != 0 &&
            id.length == id.rows * id.cols * id.depth &&
            data != nullptr;
    };
    typedef struct {
        ImageData img[2];
    } StereoImageData;

    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix34_t matPose );
    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix44_t matPose );
}

#endif
