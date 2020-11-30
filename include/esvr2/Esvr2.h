#ifndef _Esvr2_H_
#define _Esvr2_H_

#include "OgreMatrix4.h"
#include "openvr.h"
#include "opencv2/opencv.hpp"
#include <Threading/OgreThreads.h>
#include <Threading/OgreBarrier.h>

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
    //TODO: rename to VideoInput
    typedef enum {
        IT_NONE,
        IT_VIDEO_LOW_LATENCY,
        IT_VIDEO_OPENCV,
        IT_VIDEO_BLACKMAGIC
    } InputType;
    typedef enum {
        VRT_TO_SQUARE,
        VRT_TO_2D_RECTANGLE
    } VideoRenderTarget;
    //TODO: rename to VideoFormat or something else
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
        RIT_STEREO_SPLIT,
        RIT_STEREO_SPLIT_RAW
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
        std::string eye_str = "";
        int width = 0;
        int height = 0;
        std::vector<float> D = { 0, 0, 0, 0, 0 };
        std::vector<float> K = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::vector<float> R = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::vector<float> P = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        bool valid()
        {
            //TODO: check maybe also the other parameters
            return width != 0 && height != 0;
        };
    } CameraConfig;
    typedef struct {
        CameraConfig  leftCameraConfig = CameraConfig();
        CameraConfig  rightCameraConfig = CameraConfig();
        CameraConfig* cfg[2] = {&leftCameraConfig, &rightCameraConfig};
        float leftToRight = 0;
    } StereoCameraConfig;
    //similar implementation to cv::Mat*
    typedef struct {
        //-1 invalid frame
        int seq = -1;
        size_t length = 0;
        size_t height = 0;
        size_t width = 0;
        size_t depth = 0;
        Ogre::uint8 *data = nullptr;
        bool valid()
        {
            return seq != -1 && length != 0 &&
                length == height * width * depth &&
                data != nullptr;
        };
    } ImageData;
    typedef struct {
        ImageData img[2];
    } StereoImageData;

    typedef struct {
        bool showVideo = true;
        bool showOgreDialog = false;
        bool multithreading = false;
        bool isStereo = true;
        int screen = 0;
        WorkspaceType workspaceType = WS_TWO_CAMERAS_STEREO;
        VideoRenderTarget videoRenderTarget = VRT_TO_SQUARE;
        HmdConfig hmdConfig = HmdConfig();
    } Esvr2Config;

    typedef struct {
        InputType inputType = IT_NONE;
        VideoInputType videoInputType = VIT_NONE;
        std::string path = "";
        bool isStereo = true;
        Distortion distortion = DIST_RAW;
        bool wait4CameraConfig = false;
        StereoCameraConfig stereoCameraConfig = StereoCameraConfig();
    } Esvr2VideoInputConfig;

    VideoInputType getVideoInputType(std::string input_str);
    RosInputType getRosInputType(std::string input_str);
    InputType getInputType(std::string input_str);
    VideoRenderTarget getRenderVideoTarget(std::string input_str);
    WorkspaceType getWorkspaceType(std::string workspace_str);
    Distortion getDistortionType( std::string distortion_str );

    unsigned long renderThread(Ogre::ThreadHandle *threadHandle);
    unsigned long logicThread(Ogre::ThreadHandle *threadHandle);

    class VideoLoader;
    class LaparoscopeController;
    class PoseState;
    class GraphicsSystem;
    class GameState;

    class Esvr2 {
        friend GraphicsSystem;
        friend GameState;
        friend unsigned long renderThread(Ogre::ThreadHandle *threadHandle);
        friend unsigned long logicThread(Ogre::ThreadHandle *threadHandle);
    public:
        Esvr2( std::shared_ptr<Esvr2Config> config,
               std::shared_ptr<VideoLoader> videoLoader,
               std::shared_ptr<LaparoscopeController> laparoscopeController,
               std::shared_ptr<PoseState> poseState);
        ~Esvr2();
        int run();

    private:
        unsigned long renderThread1();
        unsigned long logicThread1();
    private:

        bool mIsConfigured;
        std::shared_ptr<Esvr2Config> mConfig;
        std::shared_ptr<VideoLoader> mVideoLoader;
        std::shared_ptr<LaparoscopeController> mLaparoscopeController;
        std::shared_ptr<PoseState> mPoseState;

        std::shared_ptr<GraphicsSystem> mGraphicsSystem;
        std::shared_ptr<GameState> mGameState;

        Ogre::Barrier mBarrier;
        Ogre::ThreadHandlePtr mThreadHandles[2];
    };

    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix34_t matPose );
    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix44_t matPose );

}

#endif