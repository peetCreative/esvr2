
#include "GraphicsSystem.h"
#include "GameState.h"
#include "LogicSystem.h"
#include "StereoRendering.h"
#include "StereoRenderingGameState.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRenderingOpenCvVideoLoader.h"
#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingROSNode.h"

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

#include "System/MainEntryPoints.h"

#include <experimental/filesystem>
#include <libconfig.h++>

extern const double cFrametime;
const double cFrametime = 1.0 / 25.0;

using namespace libconfig;
using namespace Demo;
using namespace esvr2;

namespace fs = std::experimental::filesystem;

struct ThreadData
{
    StereoGraphicsSystem  *graphicsSystem;
    StereoRenderingGameState  *graphicsGameState;
    VideoLoader     *videoSource;
    InputType       inputType;
    CameraConfig    *cameraConfig;
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

RosInputType getRosInputType(std::string input_str)
{
    RosInputType input = ROS_NONE;
    if (input_str.compare("MONO") == 0)
        input = ROS_MONO;
    if (input_str.compare("STEREO_SLICED") == 0)
        input = ROS_STEREO_SLICED;
    if (input_str.compare("STEREO_SPLIT") == 0)
        input = ROS_STEREO_SPLIT;
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

VideoRenderTarget getRenderVideoTarget(std::string input_str)
{
    VideoRenderTarget input = TO_SQUARE;
    if (input_str.compare("TO_SQUARE") == 0)
        input = TO_SQUARE;
    if (input_str.compare("TO_BACKGROUND") == 0)
        input = TO_BACKGROUND;
    return input;
}

WorkspaceType getWorkspaceType(std::string workspace_str)
{
    WorkspaceType workspace = WS_TWO_CAMERAS_STEREO;
    if( workspace_str.compare( "WS_TWO_CAMERAS_STEREO" ) )
        workspace = WS_TWO_CAMERAS_STEREO;
    if( workspace_str.compare( "WS_TWO_CAMERAS_STEREO" ) )
        workspace = WS_INSTANCED_STEREO;
    return workspace;
}


int main( int argc, char *argv[] )
{
    bool show_ogre_dialog = false;
    bool show_video = true;
    bool multiThreading = false;
    int screen = 0;
    bool isStereo = false;
    WorkspaceType workspace = WS_TWO_CAMERAS_STEREO;
    InputType input = NONE;
    VideoRenderTarget renderVideoTarget = TO_SQUARE;
    size_t config_files_end = 0, config_files_begin = 0;
    CameraConfig *cameraConfig = nullptr;
//     std::cout << config_file << std::endl;
    //TODO: strangely vrData needs this but hmdConfig needs initialized list
    const Ogre::Matrix4 id[2] = { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY };
    HmdConfig hmdConfig{
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45}},
        1920, 1080 };
    //TODO:That's not the most beautifaul solution charing vrData with GameState
    Ogre::VrData *vrData = new Ogre::VrData();
    vrData->set( id, id );
    VideoInput videoInput;
    videoInput.videoInputType = VIDEO_NONE;
    videoInput.path = "";
    RosInputType rosInputType = ROS_NONE;
    for (int i = 1; i < argc; i++)
    {
        //TODO check we are not using ros commands
        if ( std::strcmp(argv[i], "--config") == 0 && i+1 < argc )
        {
            i++;
            config_files_begin = i;
            config_files_end = i;
            while( i < argc && std::strncmp(argv[i], "--", 2) != 0 )
            {
                config_files_end++;
                i++;
            }
            if( i >= argc )
                break;
        }
        if ( std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc )
        {
            input = getInputType(argv[i+1]);
        }
        if ( std::strcmp(argv[i], "--video-path") == 0 && i+1 < argc )
        {
            videoInput.path = std::string(argv[i+1]);
            input = VIDEO;
        }
        if ( std::strcmp(argv[i], "--show-ogre-dialog") == 0 )
            show_ogre_dialog = true;
        if ( std::strcmp(argv[i], "--multithreading") == 0 )
            multiThreading = true;
    }

    std::vector<std::string> config_files{};
    for (size_t i = config_files_begin; i < config_files_end; i++ )
    {
        std::string path_str(argv[i]);
        if ( path_str.back() == '/' )
        {
            for( auto& p: fs::directory_iterator(path_str) )
            {
                config_files.push_back( p.path() );
            }
        }
        else
        {
            config_files.push_back( path_str );
        }
    }
    //from config file
    Config cfg;
    try
    {
        for ( auto& config_file: config_files )
        {
            LOG << "read from config file: " << config_file << LOGEND;
            cfg.readFile( config_file.c_str() );

            if (cfg.exists("show_video"))
                cfg.lookupValue ("show_video", show_video);
            if (cfg.exists("show_ogre_dialog"))
                cfg.lookupValue ("show_ogre_dialog", show_ogre_dialog);
            if (cfg.exists("multithreading"))
                cfg.lookupValue ("multithreading", multiThreading);
            if (cfg.exists("screen"))
                cfg.lookupValue ("screen", screen);
            if (cfg.exists("workspace_type"))
            {
                std::string workspace_str;
                cfg.lookupValue("workspace_type", workspace_str);
                workspace = getWorkspaceType(workspace_str);
            }
            if (cfg.exists("render_video_target"))
            {
                std::string input_str;
                cfg.lookupValue("render_video_target", input_str);
                renderVideoTarget = getRenderVideoTarget(input_str);
            }
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
                if (vs.exists("input_type"))
                {
                    std::string video_input_str;
                    video_input_str = vs["input_type"].c_str();
                    videoInput.videoInputType =
                        getVideoInputType(video_input_str);
                    isStereo = videoInput.videoInputType > VIDEO_MONO;
                }
            }

            //ROS
            if (input == ROS && cfg.exists("ros"))
            {
                Setting& vs = cfg.lookup("ros");
                if (vs.exists("input_type"))
                {
                    std::string ros_input_str;
                    ros_input_str = vs["input_type"].c_str();
                    rosInputType =
                        getRosInputType(ros_input_str);
                    isStereo = rosInputType > ROS_MONO;
                }
            }

            std::string categories[2] = {"left", "right"};
            //CAMERA_CONFIG
            if (cfg.exists("camera_info.left") &&
                cfg.exists("camera_info.right"))
            {
                cameraConfig = new CameraConfig();
                const Setting& cis = cfg.lookup("camera_info");
                if (cis.exists("left_to_right"))
                {
                    cameraConfig->leftToRight = cis["left_to_right"];
                }
                for (int leftOrRight = 0; leftOrRight < 2; leftOrRight++)
                {
                    Setting& s = cis.lookup(categories[leftOrRight]);
                    if (s.exists("width") && s.exists("height"))
                    {
                        //I'm not sure I have seen such a shit lib like libconfig++
                        //lookupValue gives me back 0 because it is parse point sth to 0 SHIT!!!!!
                        cameraConfig->width[leftOrRight] = s["width"];
                        cameraConfig->height[leftOrRight] = s["height"];
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
                if (cis.exists("width"))
                {
                    hmdConfig.width = cis["width"];
                }
                if (cis.exists("height"))
                {
                    hmdConfig.height = cis["height"];
                }
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

    StereoRenderingGameState *graphicsGameState =
        new StereoRenderingGameState(
            "Description of what we are doing", isStereo, vrData );

    StereoGraphicsSystem *graphicsSystem = new StereoGraphicsSystem(
            graphicsGameState, workspace, vrData,
            hmdConfig, screen, isStereo, show_ogre_dialog,
            show_video, renderVideoTarget );

    graphicsGameState->_notifyGraphicsSystem( graphicsSystem );

    VideoLoader *videoLoader = nullptr;
    switch(input)
    {
        case VIDEO:
        //TODO: GraphicsSystem
            videoLoader = new OpenCvVideoLoader( graphicsSystem, videoInput );
            graphicsSystem->_notifyVideoSource( videoLoader );
            break;
        case ROS:
#ifdef USE_ROS
        //TODO: GraphicsSystem
            videoLoader = new VideoROSNode(
                graphicsSystem, argc, argv, rosInputType );
            graphicsSystem->_notifyVideoSource( videoLoader );
            break;
#endif
        case NONE:
            delete graphicsGameState;
            delete graphicsSystem;
            delete videoLoader;
            LOG << "no input: shutdown" << LOGEND;
            return 1;
    }

    if ( multiThreading )
    {
        LOG << "multiThreading" << LOGEND;
        Ogre::Barrier *barrier = new Ogre::Barrier( 2 );
        ThreadData *threadData = new ThreadData();
        threadData->graphicsSystem   = graphicsSystem;
        threadData->graphicsGameState = graphicsGameState;
        threadData->videoSource      = videoLoader;
        threadData->inputType        = input;
        threadData->cameraConfig     = cameraConfig;
        threadData->barrier          = barrier;

        Ogre::ThreadHandlePtr threadHandles[2];
        threadHandles[0] = Ogre::Threads::CreateThread( THREAD_GET( renderThread1 ), 0, threadData );
        threadHandles[1] = Ogre::Threads::CreateThread( THREAD_GET( logicThread1 ), 1, threadData );

        LOG << "Render Tread " << threadHandles[0]->getThreadIdx() << LOGEND;
        LOG << "Video Source Tread " << threadHandles[1]->getThreadIdx() << LOGEND;

        Ogre::Threads::WaitForThreads( 2, threadHandles );
        delete threadData;
    }
    //SINGLETHREADED
    else
    {
        LOG << "singleThreading" << LOGEND;
        graphicsSystem->initialize( "esvr2" );
        videoLoader->initialize();
        if ( cameraConfig )
        {
            graphicsSystem->calcAlign( *cameraConfig );
            graphicsGameState->calcAlign( *cameraConfig, 2.0f );
        }

        if( !graphicsSystem->getQuit() )
        {
            graphicsSystem->createScene01();

            Ogre::Window *renderWindow = graphicsSystem->getRenderWindow();

            Ogre::Timer timer;

            Ogre::uint64 startTime = timer.getMicroseconds();

            double timeSinceLast = 1.0 / 60.0;

            while( !graphicsSystem->getQuit() )
            {
                videoLoader->beginFrameParallel();
                graphicsSystem->beginFrameParallel();
                videoLoader->update( timeSinceLast );
                graphicsSystem->update( timeSinceLast );
                graphicsSystem->finishFrameParallel();
                videoLoader->finishFrameParallel();

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

            graphicsSystem->destroyScene();
        }
        graphicsSystem->deinitialize();
        videoLoader->deinitialize();
    }

    delete graphicsGameState;
    delete graphicsSystem;
    delete videoLoader;

    return 0;
}

//---------------------------------------------------------------------
unsigned long renderThread1( Ogre::ThreadHandle *threadHandle )
{
    ThreadData *threadData = reinterpret_cast<ThreadData*>( threadHandle->getUserParam() );
    StereoGraphicsSystem *graphicsSystem  = threadData->graphicsSystem;
    StereoRenderingGameState *graphicsGameState  = threadData->graphicsGameState;
    Ogre::Barrier *barrier          = threadData->barrier;
    CameraConfig *cameraConfig      = threadData->cameraConfig;

    graphicsSystem->initialize( "esvr2" );
    if( cameraConfig )
    {
        graphicsSystem->calcAlign( *cameraConfig );
        graphicsGameState->calcAlign( *cameraConfig, 2.0f );
    }
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
    Demo::GraphicsSystem *graphicsSystem  = threadData->graphicsSystem;
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


namespace esvr2 {
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

namespace Demo
{
    // Ihave to declare this so it compiles with std ogre
    void MainEntryPoints::createSystems( GameState **outGraphicsGameState, GraphicsSystem **outGraphicsSystem,
                                GameState **outLogicGameState, LogicSystem **outLogicSystem ) {}
    void MainEntryPoints::destroySystems( GameState *graphicsGameState, GraphicsSystem *graphicsSystem,
                                GameState *logicGameState, LogicSystem *logicSystem ) {}
    const char* MainEntryPoints::getWindowTitle(void) { return nullptr; }

}
