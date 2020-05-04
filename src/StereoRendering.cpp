
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

//Declares WinMain / main
#include "MainEntryPointHelper.h"
#include "System/MainEntryPoints.h"

Ogre::String global_config_str_ = "";
Ogre::String global_video_file_str_ = "";
bool global_show_config_ = false;
Demo::InputType global_input_ = Demo::NONE;

using namespace Demo;

InputType getInputType(std::string input_str)
{
    InputType input = NONE;
    if (input_str.compare("VIDEO") == 0)
        input = VIDEO;
    if (input_str.compare("ROS") == 0)
        input = ROS;
    return input;
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
INT WINAPI WinMainApp( HINSTANCE hInst, HINSTANCE hPrevInstance, LPSTR strCmdLine, INT nCmdShow )
#else
int mainApp( int argc, const char *argv[] )
#endif
{
    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--config") == 0 && i+1 < argc)
        {
            global_config_str_ = argv[i+1];
        }
        if (std::strcmp(argv[i], "--input-type") == 0 && i+1 < argc)
        {
            global_input_ = getInputType(argv[i+1]);
        }
        if (std::strcmp(argv[i], "--video-path") == 0 && i+1 < argc)
        {
            global_video_file_str_ = std::string(argv[i+1]);
            global_input_ = VIDEO;
        }
        if(std::strcmp(argv[i], "--show-ogre-dialog") == 0)
            global_show_config_ = true;
    }
    return Demo::MainEntryPoints::mainAppSingleThreaded( DEMO_MAIN_ENTRY_PARAMS );
}

namespace Demo {
    void MainEntryPoints::createSystems(
        GameState **outGraphicsGameState,
        GraphicsSystem **outGraphicsSystem,
        GameState **outLogicGameState,
        LogicSystem **outLogicSystem )
    {
        StereoRenderingGameState *graphicsGameState = new StereoRenderingGameState(
            "Description of what we are doing" );

        std::cout << global_config_str_ << std::endl;
        HmdConfig hmdConfig{
            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
            { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45} }
        };

        GraphicsSystem *graphicsSystem = new StereoGraphicsSystem(
            graphicsGameState, WS_TWO_CAMERAS_STEREO, hmdConfig );
//         GraphicsSystem *graphicsSystem = new StereoGraphicsSystem( graphicsGameState, WS_INSTANCED_STEREO, hmdConfig );
        *outGraphicsGameState = graphicsGameState;
        *outGraphicsSystem = graphicsSystem;

        graphicsGameState->_notifyGraphicsSystem( graphicsSystem );

        LogicSystem *logicSystem;

        if ( global_input_ == Demo::VIDEO )
        {
            VideoInput vInput;
            vInput.path = global_video_file_str_;
            VideoLoader *logicGameState = new VideoLoader( vInput );
            logicSystem = new LogicSystem( logicGameState );
            logicGameState->_notifyLogicSystem( logicSystem );
            *outLogicGameState = logicGameState;
            *outLogicSystem = logicSystem;
        }
//         if ( global_input_ == Demo::ROS )
//         {
//             logicGameState = new ROSNode( ROSCOnfigoder so );
//         }

        if ( logicSystem )
        {
            graphicsSystem->_notifyLogicSystem( logicSystem );
            logicSystem->_notifyGraphicsSystem( graphicsSystem );
        }
    }

    void MainEntryPoints::destroySystems( GameState *graphicsGameState,
                                          GraphicsSystem *graphicsSystem,
                                          GameState *logicGameState,
                                          LogicSystem *logicSystem )
    {
        delete graphicsSystem;
        delete graphicsGameState;
    }

    const char* MainEntryPoints::getWindowTitle(void)
    {
        return "Stereo Rendering Sample";
    }

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
