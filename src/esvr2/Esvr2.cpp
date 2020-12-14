#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2LowLatencyVideoLoader.h"
#include "Esvr2TestPose.h"
#include "Esvr2KeyboardController.h"

#include "OgreCamera.h"
#include "OgreWindow.h"

#include "OgreTimer.h"

#include "Threading/OgreThreads.h"

#include <experimental/filesystem>
#include <mutex>
#include <memory>

namespace esvr2 {
    extern const double cFrametime;
    const double cFrametime = 1.0 / 25.0;

    THREAD_DECLARE( renderThread );
    THREAD_DECLARE( logicThread );

    ControllerType getControllerType(std::string input_str)
    {
        ControllerType controllerType = CT_NONE;
        if (input_str == "KEYBOARD")
            controllerType = CT_KEYBOARD;
        return controllerType;
    }

    VideoInputType getVideoInputType(std::string input_str)
    {
        VideoInputType input = VIT_NONE;
        if (input_str =="MONO")
            input = VIT_MONO;
        if (input_str  =="STEREO_SLICED")
            input = VIT_STEREO_SLICED;
        if (input_str =="STEREO_VERTICAL_SPLIT")
            input = VIT_STEREO_VERTICAL_SPLIT;
        if (input_str =="STEREO_HORIZONTAL_SPLIT")
            input = VIT_STEREO_HORIZONTAL_SPLIT;
        return input;
    }

    RosInputType getRosInputType(std::string input_str)
    {
        RosInputType input = RIT_NONE;
        if (input_str == "MONO")
            input = RIT_MONO;
        if (input_str == "STEREO_SLICED")
            input = RIT_STEREO_SLICED;
        if (input_str == "STEREO_SPLIT")
            input = RIT_STEREO_SPLIT;
        if (input_str == "STEREO_SPLIT_RAW")
            input = RIT_STEREO_SPLIT_RAW;
        return input;
    }

    InputType getInputType(std::string input_str)
    {
        InputType input = IT_NONE;
        if (input_str == "VIDEO_OPENCV")
            input = IT_VIDEO_OPENCV;
        if (input_str == "VIDEO_LOW_LATENCY")
            input = IT_VIDEO_LOW_LATENCY;
        if (input_str == "VIDEO_BLACKMAGIC")
            input = IT_VIDEO_BLACKMAGIC;
        return input;
    }

    VideoRenderTarget getRenderVideoTarget(std::string input_str)
    {
        VideoRenderTarget input = VRT_TO_SQUARE;
        if (input_str == "TO_SQUARE")
            input = VRT_TO_SQUARE;
        if (input_str == "TO_2D_RECTANGLE")
            input = VRT_TO_2D_RECTANGLE;
        return input;
    }

    WorkspaceType getWorkspaceType(std::string workspace_str)
    {
        WorkspaceType workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str == "TWO_CAMERAS_STEREO" )
            workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str == "INSTANCED_STEREO" )
            workspace = WS_INSTANCED_STEREO;
        return workspace;
    }

    Distortion getDistortionType( std::string distortion_str )
    {
        Distortion distortion = DIST_RAW;
        if( distortion_str == "RAW" )
            distortion = DIST_RAW;
        if( distortion_str == "UNDISTORT" )
            distortion = DIST_UNDISTORT;
        if( distortion_str == "UNDISTORT_RECTIFY" )
            distortion = DIST_UNDISTORT_RECTIFY;
        return distortion;
    }

    Esvr2::Esvr2(
        std::shared_ptr<Esvr2Config> config,
        std::shared_ptr<VideoLoader> videoLoader,
        std::shared_ptr<LaparoscopeController> laparoscopeController,
        std::shared_ptr<PoseState> poseState):
            mConfig(config),
            mVideoLoader(videoLoader),
            mController(nullptr),
            mLaparoscopeController(laparoscopeController),
            mPoseState(poseState),
            mBarrier(2)
    {
        if ( !mVideoLoader || ! mVideoLoader->initialize() )
        {
            LOG << "no videoloader or could not be initialized, Quitting" << LOGEND;
            return;
        }

        if ( !poseState )
        {
            mPoseState = std::make_shared<TestPose>();
            LOG << "no PoseState, using TestPose" << LOGEND;
        }

        // cycle until videoLoader is finished or quits
        while( !mVideoLoader->isReady() )
        {
            //TODO: look we can quit here
            Ogre::Threads::Sleep( 50 );
            videoLoader->update();
            if (videoLoader->getQuit())
            {
                LOG << "quit videoLoader" << LOGEND;
                return;
            }
        }


        mGraphicsSystem = std::make_shared<GraphicsSystem>( this );

        //TODO implement UI
        //mUi =

        mGraphicsSystem->initialize();

        switch(mConfig->controllerType)
        {
            case CT_KEYBOARD:
                mController =
                        std::make_shared<KeyboardController>(mLaparoscopeController);
                break;
//            case LC_OPT1:
//                mLaparoscopeController =
        }


        //TODO: check configuration
        mIsConfigured = true;



    }

    Esvr2::~Esvr2()
    {

    }

    bool Esvr2::getQuit()
    {
        return mGraphicsSystem->getQuit() ||
            mVideoLoader->getQuit();
    }

    int Esvr2::run()
    {
        if ( ! mIsConfigured )
            return 1;
        if ( mConfig->multithreading )
        {
            LOG << "multiThreading" << LOGEND;
            Ogre::ThreadHandlePtr mThreadHandles[2];
            mThreadHandles[0] = Ogre::Threads::CreateThread(
                    THREAD_GET( renderThread ), 0, this );
            mThreadHandles[1] = Ogre::Threads::CreateThread(
                    THREAD_GET( logicThread ), 1, this );

            LOG << "Render Tread " << mThreadHandles[0]->getThreadIdx() << LOGEND;
            LOG << "Video Source Tread " << mThreadHandles[1]->getThreadIdx() << LOGEND;

            Ogre::Threads::WaitForThreads( 2, mThreadHandles );
        }
            //SINGLETHREADED
        else
        {
            LOG << "singleThreading" << LOGEND;

            Ogre::Timer timer;

            Ogre::uint64 startTime = timer.getMicroseconds();

            double timeSinceLast = 1.0 / 60.0;

            while( !mGraphicsSystem->getQuit() )
            {
                mVideoLoader->update();
                mGraphicsSystem->update( timeSinceLast );

                if( !mGraphicsSystem->isRenderWindowVisible() )
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

            //This is not doing any thing
            mGameState->destroyScene();
            mGraphicsSystem->deinitialize();
            mVideoLoader->deinitialize();
        }
        return 0;
    }

    unsigned long renderThread(Ogre::ThreadHandle *threadHandle)
    {
        Esvr2 *esvr2 = reinterpret_cast<Esvr2*>( threadHandle->getUserParam() );
        return esvr2->renderThread1();
    }
    //---------------------------------------------------------------------
    unsigned long Esvr2::renderThread1()
    {
        Ogre::Timer timer;

        Ogre::uint64 startTime = timer.getMicroseconds();

        Ogre::uint64 timeSinceLast = 1;

        while( !getQuit() )
        {
            mGraphicsSystem->update( timeSinceLast );

            if( !mGraphicsSystem->isRenderWindowVisible() )
            {
                //Don't burn CPU cycles unnecessary when we're minimized.
                Ogre::Threads::Sleep( 500 );
            }

            Ogre::uint64 endTime = timer.getMicroseconds();
            timeSinceLast = endTime - startTime;
            startTime = endTime;
        }
        LOG << "END GRAPHICS" << LOGEND;

        mGameState->destroyScene();
        mBarrier.sync();

        mGraphicsSystem->deinitialize();
        mBarrier.sync();

        return 0;
    };

    unsigned long logicThread(Ogre::ThreadHandle *threadHandle)
    {
        Esvr2 *esvr2 = reinterpret_cast<Esvr2*>( threadHandle->getUserParam() );
        return esvr2->logicThread1();
    }


    //---------------------------------------------------------------------
    unsigned long Esvr2::logicThread1()
    {
        Ogre::Timer timer;
        //TODO: compansate YieldTimer
        //    Demo::YieldTimer yieldTimer( &timer );

        Ogre::uint64 startTime = timer.getMicroseconds();

        while( !mVideoLoader->getQuit() && !mGraphicsSystem->getQuit() )
        {
            mVideoLoader->update( );

            if( !mGraphicsSystem->isRenderWindowVisible() )
            {
                //Don't burn CPU cycles unnecessary when we're minimized.
                Ogre::Threads::Sleep( 500 );
            }

            //YieldTimer will wait until the current time is greater than startTime + cFrametime
            //TODO: compansate yield Timer
    //        startTime = yieldTimer.yield( cFrametime, startTime );
        }

        mBarrier.sync();

        mVideoLoader->deinitialize();
        mBarrier.sync();

        return 0;
    };

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
