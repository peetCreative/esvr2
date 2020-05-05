
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

//Declares WinMain / main
// #include "MainEntryPointHelper.h"
// #include "System/MainEntryPoints.h"

extern const double cFrametime;
const double cFrametime = 1.0 / 25.0;

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

    Ogre::Barrier *barrier = new Ogre::Barrier( 2 );

    StereoRenderingGameState *graphicsGameState =
        new StereoRenderingGameState("Description of what we are doing");

//     std::cout << config_file << std::endl;
    HmdConfig hmdConfig{
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
        { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45} }
    };

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
