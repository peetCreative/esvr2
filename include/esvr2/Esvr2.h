#ifndef _Esvr2_H_
#define _Esvr2_H_

#include <boost/function.hpp>

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <iostream>

// Use LEFT instead of 0
#define LEFT 0
// Use RIGHT instead of 1
#define RIGHT 1
#define MONO 0
// Define a standard for outputting messages from VR Application
#define LOG std::cout << "[esvr2] "
// Define a standard for outputting error from VR Application
#define LOG_ERROR std::cout << "[esvr2] *ERROR* "
// standard methode for end a log line
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
    //! \brief Defines Videoinput
    typedef enum {
        IT_NONE,
        IT_VIDEO_LOW_LATENCY,
        IT_VIDEO_OPENCV,
        IT_VIDEO_BLACKMAGIC
    } InputType;
    //! \deprecated allways use VRT_TO_SQUARE
    typedef enum {
        VRT_TO_SQUARE,
        VRT_TO_2D_RECTANGLE
    } VideoRenderTarget;
    //! \brief Type defines how to interpreste the incoming images from videoloader
    /*! \TODO: rename to VideoFormat or something else
     */
    typedef enum {
        VIT_NONE,
        VIT_MONO,
        VIT_STEREO_SLICED,
        VIT_STEREO_VERTICAL_SPLIT,
        VIT_STEREO_HORIZONTAL_SPLIT
    } VideoInputType;
    //! \brief Defines wether the images from Videoloader should be projected  with Undistortion and/or be rectified
    typedef enum {
        DIST_RAW,
        DIST_UNDISTORT,
        DIST_UNDISTORT_RECTIFY
    } Distortion;
    //! \brief Used when images are split horizontally or vertically
    typedef enum {
        ORIENTATION_VERTICAL,
        ORIENTATION_HORIZONTAL
    } Orientation;
    /*! whether to use in compositor the possibility of instanced Stereo,
     * which might be little faster.
     * However it's hard to render objects only in one side with this methode
     * Therefore we are using multiple compositor passes
     */
    typedef enum {
        WS_TWO_CAMERAS_STEREO,
        WS_INSTANCED_STEREO
    } WorkspaceType;
    typedef std::vector<Real> RealVector;
    typedef std::array<Real,3> RealArray3;
    typedef std::array<Real,4> RealArray4;
    typedef std::array<Real,16> RealArray16;
    //! \brief Config for the used HMD
    /* storing the projection Matrix and Transformations
     */
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
        //! \brief checks if projectionMatrix and tan is set correct
        //! \return valid
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
    /*! Camera Config of the used Laparoscope Cameras
     */
    typedef struct {
        std::string eye_str = "";
        int width = 0;
        int height = 0;
        //! Distortion parameteres
        RealVector D = { 0, 0, 0, 0, 0 };
        //! Camera Matrix
        std::array<Real, 9> K = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        //! Rectification Matrix
        std::array<Real, 9> R = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        //! Projection Matrix
        std::array<Real, 12> P = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        //! \brief check if the with and height is not null
        /* TODO: check maybe also the other parameters
         */
        bool valid()
        {
            return width != 0 && height != 0;
        };
    } CameraConfig;
    //! \brief Structure to hold stereo Camera configs
    typedef struct {
        //! left camera config
        CameraConfig  leftCameraConfig = CameraConfig();
        //! right camera config
        CameraConfig  rightCameraConfig = CameraConfig();
        //! use this to get pointers of the left and right CameraConfigs
        CameraConfig* cfg[2] = {&leftCameraConfig, &rightCameraConfig};
        //! Distance from left to right
        Real leftToRight = 0;
    } StereoCameraConfig;
    //! \brief Structure to hold so the data and meta data of images from VideoLoader
    /*! similar implementation to cv::Mat*
     */
    typedef struct {
        //! seqeunce number
        //! -1 invalid frame
        int seq = -1;
        //! number of bytes
        size_t length = 0;
        size_t height = 0;
        size_t width = 0;
        size_t depth = 0;
        //! pointer to data
        unsigned char *data = nullptr;
        //! \brief check if the structure is valid
        bool valid()
        {
            return seq != -1 && length != 0 &&
                length == height * width * depth &&
                data != nullptr;
        };
    } ImageData;
    //! \brief hold two imagedata structures
    typedef struct {
        ImageData img[2];
    } StereoImageData;
    /*! Structure to hold varius config to be passed around
     */
    typedef struct {
        //! whether VideoLoader provides images
        bool showVideo = true;
        //! whether to show the ogre dialog before starting the application
        bool showOgreDialog = false;
        //! enable/disable multithreading
        bool multithreading = false;
        //! meaningless because it's always stereo
        bool isStereo = true;
        //! wether to first run through tutorial
        bool startWithSetup = true;
        //! wether to render to screen from debugging position
        bool debugScreen = false;
        //! start application at specific screen number
        int screen = 0;
        //! initial head hight if we don't have information from HMD
        Real headHight = 1.7f;
        //! the threshold to move the head and not beeing centered anymore
        Real centerEpsilon = 0.1;
        //! Distance of the virtual projection plane at the beginning
        Real initialProjectionPlaneDistance = 2.0f;
        //! Distance of the last time the application was started projectionplane distance
        Real cachedProjectionPlaneDistance = 2.0f;
        //! minimal Distance to projection plane to configure
        Real projectionPlaneMinDistance = 0.4f;
        //! max Distance to projection plane to configure
        Real projectionPlaneMaxDistance = 10.0f;
        //! cache where the projection plane has been
        RealArray4 cachedProjectionPlaneOrientation = {0.0f, 0.0f, 0.0f, 0.0f};
        //! Distance of the menu projection plane
        Real infoScreenDistance = 1.0f;
        //! how many micro secs to wait before send new command in opt 0 and 1
        int ctlDelay = 25;
        //! the laparoscope is 30° tilted
        Real ctlCameraTilt = 0.52359;
        //! the distance to focus
        Real ctlFocusDistance = 0.2;
        //! stepwidth of in yaw in rad
        Real ctlStepYaw = 0.01;
        //! stepwidth of in pitch in rad
        Real ctlStepPitch = 0.01;
        //! stepwidth of in roll in rad
        Real ctlStepRoll = 0.01;
        //! stepwidth of in trans_z in rad
        Real ctlStepTransZ = 0.001;
        Real ctlOpt0ThresholdTransZ = 0.01;
        Real ctlOpt0ThresholdYawDeg = 5;
        Real ctlOpt0ThresholdPitchDeg = 5;
        Real ctlOpt0ThresholdRollDeg = 5;
        Real ctlOpt2TransZFact = 2.0;
        //! Ogre Resources config file
        std::string resourcePath = "Resources.cfg";
        //! Folder to put config file
        std::string logFolder = ".";
        //! Prefix to put infront of log files
        std::string logPrefix = "";
        //! Prefix to put infront of log files
        std::string cachePrefixParticipantId = "";

        //! deprecated path to serial port
        std::string serialPort = "/dev/ttyUSB0";
        //! if the ProjectionPlanes are centered by the camera matrix
        bool centerProjectionPlane = false;
        //! choosen camera controller
        ControllerType controllerType = CT_NONE;
        //! \deprecated used workspace type
        WorkspaceType workspaceType = WS_TWO_CAMERAS_STEREO;
        //! \deprecated used workspace type
        VideoRenderTarget videoRenderTarget = VRT_TO_SQUARE;
        //! Configuration of the Headset
        HmdConfig hmdConfig = HmdConfig();
    } Esvr2Config;
    typedef std::shared_ptr<Esvr2Config> Esvr2ConfigPtr;

    //! \brief Structure to configure the VideoLoader also used by the projection
    typedef struct {
        InputType inputType = IT_NONE;
        VideoInputType videoInputType = VIT_NONE;
        // if from videofile path to it
        std::string path = "";
        //! if the images are coming as stereo
        bool isStereo = true;
        //! if we need to undistort/rectify the immage
        Distortion distortion = DIST_RAW;
        //! if we need to wait until the stereoCameraConfig is valid
        bool wait4CameraConfig = false;
        //! Camera Calibrations of the Stereo/Monocameras
        StereoCameraConfig stereoCameraConfig = StereoCameraConfig();
    } VideoInputConfig;
    typedef std::shared_ptr<VideoInputConfig> VideoInputConfigPtr;

    //! \brief convert string to ControllerType
    ControllerType getControllerType(std::string input_str);
    //! \brief convert string to VideoInputType
    VideoInputType getVideoInputType(std::string input_str);
    //! \brief convert string to InputType
    InputType getInputType(std::string input_str);
    //! \brief convert string to VideoRenderTarget
    VideoRenderTarget getRenderVideoTarget(std::string input_str);
    //! \brief convert string to WorkspaceType
    WorkspaceType getWorkspaceType(std::string workspace_str);
    //! \brief convert string to Distortion
    Distortion getDistortionType( std::string distortion_str );

    class VideoLoader;
    class LaparoscopeController;
    class PoseState;
    class GraphicsSystem;
    class GameState;
    class OpenVRCompositorListener;
    class Component;
    //! Mainclass for the VR-Application
    class Esvr2 {
        friend GraphicsSystem;
        friend GameState;
        friend OpenVRCompositorListener;
    public:
        //! \brief constructor of the Esvr2 Object
        //! \param config fully configured Esvr2Config
        Esvr2( Esvr2ConfigPtr config);
        //! \brief Deconstructor of the Esvr2 Object
        ~Esvr2();
        //! \brief Sets the VideoLoader object if needed
        bool setVideoLoader(
                std::shared_ptr<VideoLoader> videoLoader);
        //! \brief Sets the LaparoscopeController object
        //! to make Robot or simulated camera move
        bool setLaparoscopeController(
                std::shared_ptr<LaparoscopeController> laparoscopeController);
        //! \brief Sets the PoseState object
        //! to make the application know where the laparoscope is
        bool setPoseState(
                std::shared_ptr<PoseState> poseState);
        //! \brief Function to get the tracked head position
        /*! \param rotation Quaternion of the VR-Headset Orientation
         *  \param translation 3D-Vector of the VR-Headset Position
         *  \return success
         */
        bool getHeadPose(
                std::array<Real, 3> &translation, std::array<Real, 4> &rotation);
        //! \brief Function to register more function, which are called in the update loop
        //! \return success
        bool registerUpdateCallback(const boost::function<void(uint64)>);
        //! \brief function to start the VR-Application (main loop)
        //! \return c-style success of application
        int run();
        unsigned long updateThread(
                boost::function<void(uint64)> updateFct);
        //! \brief makes the Application stop
        void quit();

    private:
        //! \brief Checks if any component is to be stopped
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