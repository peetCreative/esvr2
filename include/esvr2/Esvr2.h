#ifndef _Esvr2_H_
#define _Esvr2_H_

#include <vector>
#include <string>
#include <memory>
#include <iostream>

#define LEFT 0
#define RIGHT 1
#define MONO 0
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
        CT_NONE,
        CT_KEYBOARD
    } ControllerType;
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
    typedef std::vector<float> RealVector;
    typedef struct {
        RealVector initialPose = RealVector(3);
        RealVector eyeToHeadLeft = RealVector(3);
        RealVector eyeToHeadRight = RealVector(3);
        RealVector *eyeToHeadPtr[2] =  {&eyeToHeadLeft, &eyeToHeadRight};
        RealVector projectionMatrixLeft = RealVector(16);
        RealVector projectionMatrixRight = RealVector(16);
        RealVector *projectionMatrixPtr[2] =
                {&projectionMatrixLeft, &projectionMatrixRight};
        RealVector tanLeft = RealVector(4);
        RealVector tanRight = RealVector(4);
        RealVector *tanPtr[2] = {&tanLeft, &tanRight};
        unsigned int width, height;
        bool valid()
        {
          return  projectionMatrixLeft[0] && projectionMatrixLeft[2] &&
                  projectionMatrixLeft[5] && projectionMatrixLeft[6] &&
                  projectionMatrixLeft[10] && projectionMatrixLeft[11] &&
                  projectionMatrixLeft[14] &&
                  projectionMatrixRight[0] && projectionMatrixRight[2] &&
                  projectionMatrixRight[5] && projectionMatrixRight[6] &&
                  projectionMatrixRight[10] && projectionMatrixRight[11] &&
                  projectionMatrixRight[14] &&
                  tanLeft[0] && tanLeft[1] && tanLeft[2] && tanLeft[3] &&
                  tanRight[0] && tanRight[1] && tanRight[2] && tanRight[3];
        };
    } HmdConfig;
    typedef struct {
        std::string eye_str = "";
        int width = 0;
        int height = 0;
        RealVector D = { 0, 0, 0, 0, 0 };
        RealVector K = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        RealVector R = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        RealVector P = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
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
        unsigned char *data = nullptr;
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
        bool debugScreen = false;
        int screen = 0;
        double headHight = 1.7f;
        std::string resourcePath = "Resources.cfg";
        ControllerType controllerType = CT_NONE;
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

    ControllerType getControllerType(std::string input_str);
    VideoInputType getVideoInputType(std::string input_str);
    InputType getInputType(std::string input_str);
    VideoRenderTarget getRenderVideoTarget(std::string input_str);
    WorkspaceType getWorkspaceType(std::string workspace_str);
    Distortion getDistortionType( std::string distortion_str );

    class VideoLoader;
    class Controller;
    class LaparoscopeController;
    class PoseState;
    class GraphicsSystem;
    class GameState;
    class OpenVRCompositorListener;
    //Really dirty wrapping Ogre::Barrier so we don't need to put Ogre in Esvr2.h
    class Barrier;

    class Esvr2 {
        friend GraphicsSystem;
        friend GameState;
        friend OpenVRCompositorListener;
    public:
        Esvr2( Esvr2Config *config,
               VideoLoader *videoLoader,
               LaparoscopeController *laparoscopeController,
               PoseState *poseState);
        Esvr2( std::shared_ptr<Esvr2Config> config,
               std::shared_ptr<VideoLoader> videoLoader,
               std::shared_ptr<LaparoscopeController> laparoscopeController,
               std::shared_ptr<PoseState> poseState);
        ~Esvr2();
        int run();
        unsigned long renderThread1();
        unsigned long logicThread1();

    private:
        bool getQuit();
    private:

        bool mIsConfigured;
        std::shared_ptr<Esvr2Config> mConfig;
        std::shared_ptr<VideoLoader> mVideoLoader;
        std::shared_ptr<Controller> mController;
        std::shared_ptr<LaparoscopeController> mLaparoscopeController;
        std::shared_ptr<PoseState> mPoseState;

        std::shared_ptr<GraphicsSystem> mGraphicsSystem;

        Barrier *mBarrier;
    };
}

#endif