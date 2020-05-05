
#include "GraphicsSystem.h"
#include "GameState.h"
#include "LogicSystem.h"
#include "StereoRendering.h"
#include "StereoRenderingGameState.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRenderingVideoLoader.h"

#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
#include "Compositor/OgreCompositorManager2.h"

#include "Threading/YieldTimer.h"

#include "OgreWindow.h"
#include "OgreTimer.h"

#include "Threading/OgreThreads.h"
#include "Threading/OgreBarrier.h"

#include <libconfig.h++>

extern const double cFrametime;
const double cFrametime = 1.0 / 25.0;

using namespace libconfig;
using namespace Demo;

struct ThreadData
{
    GraphicsSystem  *graphicsSystem;
    VideoLoader     *videoSource;
    InputType       inputType;
    Ogre::Barrier   *barrier;
};


unsigned long renderThread1( Ogre::ThreadHandle *threadHandle );
unsigned long logicThread1( Ogre::ThreadHandle *threadHandle );


THREAD_DECLARE( renderThread1 );
THREAD_DECLARE( logicThread1 );


VideoInputType getVideoInputType(std::string input_str)
{
    VideoInputType input = VIDEO_NONE;
    if (input_str.compare("MONO") == 0)
        input = VIDEO_MONO;
    if (input_str.compare("STEREO_SLICED") == 0)
        input = VIDEO_STEREO_SLICED;
    if (input_str.compare("STEREO_VERTICAL_SPLIT") == 0)
        input = VIDEO_STEREO_VERTICAL_SPLIT;
    if (input_str.compare("STEREO_HORIZONTAL_SPLIT") == 0)
        input = VIDEO_STEREO_HORIZONTAL_SPLIT;
    return input;
}

InputType getInputType(std::string input_str)
{
    InputType input = NONE;
    if (input_str.compare("VIDEO") == 0)
        input = VIDEO;
    if (input_str.compare("ROS") == 0)
        input = ROS;
    return input;
}

int main( int argc, const char *argv[] )
{
    bool show_ogre_dialog = false;
    InputType input = NONE;
    const char *config_file = nullptr;
    CameraConfig *cameraConfig = nullptr;
//     std::cout << config_file << std::endl;
    HmdConfig hmdConfig{
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45} }
    };
    VideoInput videoInput;
    videoInput.path = "";
    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--config") == 0 && i+1 < argc)
        {
            config_file = argv[i+1];
        }
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            input = getInputType(argv[i+1]);
        }
        if (std::strcmp(argv[i], "--video-path") == 0 && i+1 < argc)
        {
            videoInput.path = std::string(argv[i+1]);
            input = VIDEO;
        }
        if(std::strcmp(argv[i], "--show-ogre-dialog") == 0)
            show_ogre_dialog = true;
    }

    //from config file
    if(config_file)
    {
        Config cfg;
        try
        {
            LOG << "read from config file" << LOGEND;
            cfg.readFile(config_file);
            if (cfg.exists("show_ogre_dialog"))
                cfg.lookupValue ("show_ogre_dialog", show_ogre_dialog);
            //only set input if we didn't set it by cmdline
            if (cfg.exists("input_type") && input == NONE)
            {
                std::string input_str;
                cfg.lookupValue("input_type", input_str);
                input = getInputType(input_str);
            }
            //VIDEO
            if (input == VIDEO && cfg.exists("video"))
            {
                Setting& vs = cfg.lookup("video");
                if (vs.exists("path"))
                    videoInput.path = vs["path"].c_str();
                if (vs.exists("video_type"))
                {
                    std::string video_input_str;
                    video_input_str = vs["video_type"].c_str();
                    videoInput.videoInputType = getVideoInputType(video_input_str);
                }
            }

            std::string categories[2] = {"left", "right"};
            //CAMERA_CONFIG
            if (cfg.exists("camera_info.left") &&
                cfg.exists("camera_info.right"))
            {
                cameraConfig = new CameraConfig();
                const Setting& cis = cfg.lookup("camera_info");
                for (int leftOrRight = 0; leftOrRight < 2; leftOrRight++)
                {
                    Setting& s = cis.lookup(categories[leftOrRight]);
                    if (s.exists("width") && s.exists("height"))
                    {
                        //I'm not sure I have seen such a shit lib like libconfig++
                        //lookupValue gives me back 0 because it is parse point sth to 0 SHIT!!!!!
                        cameraConfig->width[leftOrRight] =
                            s["width"];
                        cameraConfig->height[leftOrRight] =
                            s["width"];
                    }
                    else
                        LOG << "camera_config is invalid" << LOGEND;
                    if (s.exists("f_x") && s.exists("f_y") &&
                        s.exists("c_x") && s.exists("c_y"))
                    {
                        cameraConfig->f_x[leftOrRight] = s["f_x"];
                        cameraConfig->f_y[leftOrRight] = s["f_y"];
                        cameraConfig->c_x[leftOrRight] = s["c_x"];
                        cameraConfig->c_y[leftOrRight] = s["c_y"];
                    }
                    if (s.exists("K") && s["K"].getLength() == 9)
                    {
                        cameraConfig->f_x[leftOrRight] = s["K"][0];
                        cameraConfig->f_y[leftOrRight] = s["K"][4];
                        cameraConfig->c_x[leftOrRight] = s["K"][2];
                        cameraConfig->c_y[leftOrRight] = s["K"][5];
                    }
                }
            }
            if (cfg.exists("hmd_info.left") &&
                cfg.exists("hmd_info.right"))
            {
                const Setting& cis = cfg.lookup("hmd_info");
                for (int leftOrRight = 0; leftOrRight < 2; leftOrRight++)
                {
                    Setting& s = cis.lookup(categories[leftOrRight]);
                    if (s.exists("eye_to_head") &&
                        s["eye_to_head"].getLength() == 16 &&
                        s.exists("projection_matrix") &&
                        s["projection_matrix"].getLength() == 16 &&
                        s.exists("tan") &&
                        s["tan"].getLength() == 4)
                    {
                        Setting& st = s["tan"];
                        hmdConfig.tan[leftOrRight] =
                            Ogre::Vector4(st[0], st[1], st[2], st[3]);
                        Setting& se2h = s["eye_to_head"];
                        Setting& spm = s["projection_matrix"];
                        float e2h[16];
                        memset(&e2h, 0, sizeof(float[16]));
                        float pm[16];
                        memset(&pm, 0, sizeof(float[16]));
                        for (int i = 0; i < 16; i++)
                        {
                            //This is a horrible bug or feature of this library
                            //An array is not auto casting to float,
                            //an List can contain different types arrrrrg
                            // so we use lists... and casting
                            if (se2h[i].getType() == 1)
                            {
                                int a = se2h[i];
                                e2h[i] = static_cast<float>(a);
                            }
                            else
                            {
                                e2h[i] = se2h[i];
                            }
                            if (spm[i].getType() == 1)
                            {
                                int a = spm[i];
                                pm[i] = static_cast<float>(a);
                            }
                            else
                            {
                                pm[i] = spm[i];
                            }
                        }
                        hmdConfig.eyeToHead[leftOrRight] = Ogre::Matrix4(
                            e2h[ 0], e2h[ 1], e2h[ 2], e2h[ 3],
                            e2h[ 4], e2h[ 5], e2h[ 6], e2h[ 7],
                            e2h[ 8], e2h[ 9], e2h[10], e2h[11],
                            e2h[12], e2h[13], e2h[14], e2h[15]
                            );
                        hmdConfig.projectionMatrix[leftOrRight] = Ogre::Matrix4(
                            pm[ 0], pm[ 1], pm[ 2], pm[ 3],
                            pm[ 4], pm[ 5], pm[ 6], pm[ 7],
                            pm[ 8], pm[ 9], pm[10], pm[11],
                            pm[12], pm[13], pm[14], pm[15]
                        );
                    }
                    else
                    {
                        LOG << "There is some typo in your hmd config" << LOGEND;
                    }
                }
            }
        }
        catch(const FileIOException &fioex)
        {
            std::cerr << "I/O error while reading file." << std::endl;
            return(EXIT_FAILURE);
        }
        catch(const ParseException &pex)
        {
            std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                    << " - " << pex.getError() << std::endl;
            return(EXIT_FAILURE);
        }
    }

    Ogre::Barrier *barrier = new Ogre::Barrier( 2 );

    StereoRenderingGameState *graphicsGameState =
        new StereoRenderingGameState("Description of what we are doing");

    StereoGraphicsSystem *graphicsSystem = new StereoGraphicsSystem(
            graphicsGameState, WS_TWO_CAMERAS_STEREO, hmdConfig, show_ogre_dialog );
//     GraphicsSystem *graphicsSystem = new StereoGraphicsSystem(
//         graphicsGameState, WS_INSTANCED_STEREO, hmdConfig );

    graphicsGameState->_notifyGraphicsSystem( graphicsSystem );

    VideoLoader *videoLoader = nullptr;
    if ( input == VIDEO )
    {
        //TODO: GraphicsSystem
        videoLoader = new VideoLoader( /*graphicsSystem,*/ videoInput );
        graphicsSystem->_notifyVideoSource( videoLoader );
    }

    ThreadData *threadData = new ThreadData();
    threadData->graphicsSystem   = graphicsSystem;
    threadData->videoSource      = videoLoader;
    threadData->inputType        = input;
    threadData->barrier          = barrier;

    Ogre::ThreadHandlePtr threadHandles[2];
    threadHandles[0] = Ogre::Threads::CreateThread( THREAD_GET( renderThread1 ), 0, threadData );
    threadHandles[1] = Ogre::Threads::CreateThread( THREAD_GET( logicThread1 ), 1, threadData );

    LOG << "Render Tread " << threadHandles[0]->getThreadIdx() << LOGEND;
    LOG << "Video Source Tread " << threadHandles[1]->getThreadIdx() << LOGEND;

    Ogre::Threads::WaitForThreads( 2, threadHandles );

    delete threadData;
    delete graphicsGameState;
    delete graphicsSystem;
    delete videoLoader;

    return 0;
}

//---------------------------------------------------------------------
unsigned long renderThread1( Ogre::ThreadHandle *threadHandle )
{
    ThreadData *threadData = reinterpret_cast<ThreadData*>( threadHandle->getUserParam() );
    GraphicsSystem *graphicsSystem  = threadData->graphicsSystem;
    Ogre::Barrier *barrier          = threadData->barrier;

    graphicsSystem->initialize( "esvr2" );
    barrier->sync();

    if( graphicsSystem->getQuit() )
    {
        graphicsSystem->deinitialize();
        return 0; //User cancelled config
    }

    graphicsSystem->createScene01();
    barrier->sync();

    Ogre::Window *renderWindow = graphicsSystem->getRenderWindow();

    Ogre::Timer timer;

    Ogre::uint64 startTime = timer.getMicroseconds();

    double timeSinceLast = 1.0 / 60.0;

    while( !graphicsSystem->getQuit() )
    {
        graphicsSystem->beginFrameParallel();
        graphicsSystem->update( timeSinceLast );
        graphicsSystem->finishFrameParallel();

        if( !renderWindow->isVisible() )
        {
            //Don't burn CPU cycles unnecessary when we're minimized.
            Ogre::Threads::Sleep( 500 );
        }

        Ogre::uint64 endTime = timer.getMicroseconds();
        timeSinceLast = (endTime - startTime) / 1000000.0;
        timeSinceLast = std::min( 1.0, timeSinceLast ); //Prevent from going haywire.
        startTime = endTime;
    }
    LOG << "END GRAPHICS" << LOGEND;
//     barrier->sync();

    graphicsSystem->destroyScene();
    barrier->sync();

    graphicsSystem->deinitialize();
    barrier->sync();

    return 0;
};


// unsigned long renderThread1( Ogre::ThreadHandle *threadHandle )
// {
//     unsigned long retVal = -1;
// 
//     try
//     {
//         retVal = renderThread1App( threadHandle );
//     }
//     catch( Ogre::Exception& e )
//     {
//    #if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//         MessageBoxA( NULL, e.getFullDescription().c_str(), "An exception has occured!",
//                      MB_OK | MB_ICONERROR | MB_TASKMODAL );
//    #else
//         std::cerr << "An exception has occured: " <<
//                      e.getFullDescription().c_str() << std::endl;
//    #endif
// 
//         abort();
//     }
// 
//     return retVal;
// }

//---------------------------------------------------------------------
unsigned long logicThread1( Ogre::ThreadHandle *threadHandle )
{
    ThreadData *threadData = reinterpret_cast<ThreadData*>( threadHandle->getUserParam() );
    GraphicsSystem *graphicsSystem  = threadData->graphicsSystem;
    VideoLoader *videoLoader        = threadData->videoSource;
    Ogre::Barrier *barrier          = threadData->barrier;

    videoLoader->initialize();
    barrier->sync();

    if( graphicsSystem->getQuit() )
    {
        videoLoader->deinitialize();
        return 0; //Render thread cancelled early
    }

//     videoLoader->loadVideo();
    barrier->sync();

    Ogre::Window *renderWindow = graphicsSystem->getRenderWindow();

    Ogre::Timer timer;
    YieldTimer yieldTimer( &timer );

    Ogre::uint64 startTime = timer.getMicroseconds();

    while( !graphicsSystem->getQuit() )
    {
        videoLoader->beginFrameParallel();
        videoLoader->update( static_cast<float>( cFrametime ) );
        videoLoader->finishFrameParallel();

        videoLoader->finishFrame();

        if( !renderWindow->isVisible() )
        {
            //Don't burn CPU cycles unnecessary when we're minimized.
            Ogre::Threads::Sleep( 500 );
        }

        //YieldTimer will wait until the current time is greater than startTime + cFrametime
        startTime = yieldTimer.yield( cFrametime, startTime );
    }

    barrier->sync();

    videoLoader->deinitialize();
    barrier->sync();

    return 0;
};


namespace Demo {


//         StereoRenderingGameState *graphicsGameState = new StereoRenderingGameState(
//             );
//
//         std::cout << global_config_str_ << std::endl;
//         HmdConfig hmdConfig{
//             { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
//             { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
//             { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45} }
//         };
//
//         GraphicsSystem *graphicsSystem = new StereoGraphicsSystem(
//             graphicsGameState, WS_TWO_CAMERAS_STEREO, hmdConfig );
// //         GraphicsSystem *graphicsSystem = new StereoGraphicsSystem( graphicsGameState, WS_INSTANCED_STEREO, hmdConfig );
//         *outGraphicsGameState = graphicsGameState;
//         *outGraphicsSystem = graphicsSystem;
// 
//         graphicsGameState->_notifyGraphicsSystem( graphicsSystem );
//
//         LogicSystem *logicSystem;
//
//         if ( global_input_ == Demo::VIDEO )
//         {
//             VideoInput vInput;
//             vInput.path = global_video_file_str_;
//             VideoLoader *logicGameState = new VideoLoader( vInput );
//             logicSystem = new LogicSystem( logicGameState );
//             logicGameState->_notifyLogicSystem( logicSystem );
//             *outLogicGameState = logicGameState;
//             *outLogicSystem = logicSystem;
//         }
// //         if ( global_input_ == Demo::ROS )
// //         {
// //             logicGameState = new ROSNode( ROSCOnfigoder so );
// //         }
// 
//         if ( logicSystem )
//         {
//             graphicsSystem->_notifyLogicSystem( logicSystem );
//             logicSystem->_notifyGraphicsSystem( graphicsSystem );
//         }
//     }

    //-------------------------------------------------------------------------
    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix34_t matPose )
    {
        Ogre::Matrix4 matrixObj(
                    matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
                    matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
                    matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
                               0.0f,            0.0f,            0.0f,            1.0f );
        return matrixObj;
    }

    //-------------------------------------------------------------------------
    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix44_t matPose )
    {
        Ogre::Matrix4 matrixObj(
                    matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
                    matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
                    matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
                    matPose.m[3][0], matPose.m[3][1], matPose.m[3][2], matPose.m[3][3] );
        return matrixObj;
    }

}
