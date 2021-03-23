#ifndef _Esvr2_H_
#define _Esvr2_H_

#include <boost/function.hpp>

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <iostream>

#define LEFT 0
#define RIGHT 1
#define MONO 0
#define LOG std::cout << "[esvr2] "
#define LOG_ERROR std::cout << "[esvr2] *ERROR* "
#define LOGEND std::endl

#ifndef RESOURCE_FOLDER
#define RESOURCE_FOLDER "../Data/"
#endif
#ifndef PLUGIN_FOLDER
#define PLUGIN_FOLDER "./"
#endif

namespace esvr2 {
#ifdef USE_DOUBLE_PRECISION
    typedef double Real;
#else
    typedef float Real;
#endif
    typedef unsigned long long uint64;
    typedef enum {
        CT_NONE,
        CT_OPT0,
        CT_OPT1,
        CT_OPT2
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
    typedef std::vector<Real> RealVector;
    typedef std::array<Real,3> RealArray3;
    typedef std::array<Real,4> RealArray4;
    typedef std::array<Real,16> RealArray16;
    typedef struct {
        RealArray3 initialPose = RealArray3();
        RealArray3 eyeToHeadLeft = RealArray3();
        RealArray3 eyeToHeadRight = RealArray3();
        RealArray3 *eyeToHeadPtr[2] =  {&eyeToHeadLeft, &eyeToHeadRight};
        RealArray16 projectionMatrixLeft = RealArray16();
        RealArray16 projectionMatrixRight = RealArray16();
        RealArray16 *projectionMatrixPtr[2] =
                {&projectionMatrixLeft, &projectionMatrixRight};
        RealArray4 tanLeft = RealArray4();
        RealArray4 tanRight = RealArray4();
        RealArray4 *tanPtr[2] = {&tanLeft, &tanRight};
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
        std::array<Real, 9> K = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::array<Real, 9> R = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        std::array<Real, 12> P = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
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
        Real leftToRight = 0;
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
        bool startWithSetup = true;
        bool debugScreen = false;
        int screen = 0;
        Real headHight = 1.7f;
        Real centerEpsilon = 0.1;
        Real initialProjectionPlaneDistance = 2.0f;
        Real infoScreenDistance = 1.0f;
        //the laparoscope is 30° tilted
        int ctlDelay = 25;
        Real ctlCameraTilt = 0.52359;
        Real ctlFocusDistance = 0.2;
        Real ctlStepYaw = 0.01;
        Real ctlStepPitch = 0.01;
        Real ctlStepRoll = 0.01;
        Real ctlStepTransZ = 0.001;
        Real ctlOpt0ThresholdTransZ = 0.01;
        Real ctlOpt0ThresholdYawDeg = 5;
        Real ctlOpt0ThresholdPitchDeg = 5;
        Real ctlOpt0ThresholdRollDeg = 5;
        Real ctlOpt2TransZFact = 2.0;
        std::string resourcePath = "Resources.cfg";
        std::string logFolder = ".";
        std::string logPrefix = "";
        std::string serialPort = "/dev/ttyUSB0";
        bool centerProjectionPlane = false;
        ControllerType controllerType = CT_NONE;
        WorkspaceType workspaceType = WS_TWO_CAMERAS_STEREO;
        VideoRenderTarget videoRenderTarget = VRT_TO_SQUARE;
        HmdConfig hmdConfig = HmdConfig();
    } Esvr2Config;
    typedef std::shared_ptr<Esvr2Config> Esvr2ConfigPtr;

    typedef struct {
        InputType inputType = IT_NONE;
        VideoInputType videoInputType = VIT_NONE;
        std::string path = "";
        bool isStereo = true;
        Distortion distortion = DIST_RAW;
        bool wait4CameraConfig = false;
        StereoCameraConfig stereoCameraConfig = StereoCameraConfig();
    } VideoInputConfig;
    typedef std::shared_ptr<VideoInputConfig> VideoInputConfigPtr;

    ControllerType getControllerType(std::string input_str);
    VideoInputType getVideoInputType(std::string input_str);
    InputType getInputType(std::string input_str);
    VideoRenderTarget getRenderVideoTarget(std::string input_str);
    WorkspaceType getWorkspaceType(std::string workspace_str);
    Distortion getDistortionType( std::string distortion_str );

    class VideoLoader;
    class LaparoscopeController;
    class PoseState;
    class GraphicsSystem;
    class GameState;
    class OpenVRCompositorListener;
    class Component;
    class Esvr2 {
        friend GraphicsSystem;
        friend GameState;
        friend OpenVRCompositorListener;
    public:
        Esvr2( Esvr2ConfigPtr config);
        bool setVideoLoader(
                std::shared_ptr<VideoLoader> videoLoader);
        bool setLaparoscopeController(
                std::shared_ptr<LaparoscopeController> laparoscopeController);
        bool setPoseState(
                std::shared_ptr<PoseState> poseState);
        bool getHeadPose(
                std::array<Real, 3> &translation, std::array<Real, 4> &rotation);
        bool registerUpdateCallback(const boost::function<void(uint64)>);
        ~Esvr2();
        int run();
        unsigned long updateThread(
                boost::function<void(uint64)> updateFct);
        void quit();

    private:
        bool getQuit();
    private:

        bool mIsConfigured {false};
        Esvr2ConfigPtr mConfig {nullptr};
        std::shared_ptr<VideoLoader> mVideoLoader {nullptr};
        std::shared_ptr<LaparoscopeController> mLaparoscopeController {nullptr};
        std::shared_ptr<PoseState> mPoseState {nullptr};
        std::vector<std::shared_ptr<Component>> mComponents {};
        std::vector<boost::function<void(uint64)>> mUpdateCallbacks{};

        std::shared_ptr<GraphicsSystem> mGraphicsSystem;
    };
}

#endif