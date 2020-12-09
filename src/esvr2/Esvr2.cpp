#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2LowLatencyVideoLoader.h"
#include "Esvr2TestPose.h"
#include "Esvr2LaparoscopeController.h"

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


    VideoInputType getVideoInputType(std::string input_str)
    {
        VideoInputType input = VIT_NONE;
        if (input_str.compare("MONO") == 0)
            input = VIT_MONO;
        if (input_str.compare("STEREO_SLICED") == 0)
            input = VIT_STEREO_SLICED;
        if (input_str.compare("STEREO_VERTICAL_SPLIT") == 0)
            input = VIT_STEREO_VERTICAL_SPLIT;
        if (input_str.compare("STEREO_HORIZONTAL_SPLIT") == 0)
            input = VIT_STEREO_HORIZONTAL_SPLIT;
        return input;
    }

    RosInputType getRosInputType(std::string input_str)
    {
        RosInputType input = RIT_NONE;
        if (input_str.compare("MONO") == 0)
            input = RIT_MONO;
        if (input_str.compare("STEREO_SLICED") == 0)
            input = RIT_STEREO_SLICED;
        if (input_str.compare("STEREO_SPLIT") == 0)
            input = RIT_STEREO_SPLIT;
        if (input_str.compare("STEREO_SPLIT_RAW") == 0)
            input = RIT_STEREO_SPLIT_RAW;
        return input;
    }

    InputType getInputType(std::string input_str)
    {
        InputType input = IT_NONE;
        if (input_str.compare("VIDEO_OPENCV") == 0)
            input = IT_VIDEO_OPENCV;
        if (input_str.compare("VIDEO_LOW_LATENCY") == 0)
            input = IT_VIDEO_LOW_LATENCY;
        if (input_str.compare("VIDEO_BLACKMAGIC") == 0)
            input = IT_VIDEO_BLACKMAGIC;
        return input;
    }

    VideoRenderTarget getRenderVideoTarget(std::string input_str)
    {
        VideoRenderTarget input = VRT_TO_SQUARE;
        if (input_str.compare("TO_SQUARE") == 0)
            input = VRT_TO_SQUARE;
        if (input_str.compare("TO_2D_RECTANGLE") == 0)
            input = VRT_TO_2D_RECTANGLE;
        return input;
    }

    WorkspaceType getWorkspaceType(std::string workspace_str)
    {
        WorkspaceType workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str.compare( "TWO_CAMERAS_STEREO" ) )
            workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str.compare( "INSTANCED_STEREO" ) )
            workspace = WS_INSTANCED_STEREO;
        return workspace;
    }

    Distortion getDistortionType( std::string distortion_str )
    {
        Distortion distortion = DIST_RAW;
        if( distortion_str.compare( "RAW" ) )
            distortion = DIST_RAW;
        if( distortion_str.compare( "UNDISTORT" ) )
            distortion = DIST_UNDISTORT;
        if( distortion_str.compare( "UNDISTORT_RECTIFY" ) )
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
            mLaparoscopeController(laparoscopeController),
            mPoseState(poseState),
            mBarrier(2)
    {
        if ( !videoLoader || ! videoLoader->initialize() )
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
        while( !videoLoader->isReady() )
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

        mGameState = std::make_shared<GameState>(this);

        mGraphicsSystem = std::make_shared<GraphicsSystem>( this, mGameState );

        mGameState->_notifyStereoGraphicsSystem( mGraphicsSystem );

        mGraphicsSystem->initialize( "esvr2" );
        mGameState->createLaparoscopeScene();
        mGameState->createVRScene();

        //TODO: check configuration
        mIsConfigured = true;



    }

    Esvr2::~Esvr2()
    {

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
                mGraphicsSystem->beginFrameParallel();
                mVideoLoader->update();
                mGraphicsSystem->update( timeSinceLast );
                mGraphicsSystem->finishFrameParallel();

                if( !mGraphicsSystem->getRenderWindow()->isVisible() )
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

        double timeSinceLast = 1.0 / 60.0;

        while( !mGraphicsSystem->getQuit() && !mVideoLoader->getQuit() )
        {
            mGraphicsSystem->beginFrameParallel();
            mGraphicsSystem->update( timeSinceLast );
            mGraphicsSystem->finishFrameParallel();

            if( !mGraphicsSystem->getRenderWindow()->isVisible() )
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
        Ogre::Window *renderWindow = mGraphicsSystem->getRenderWindow();

        Ogre::Timer timer;
        //TODO: compansate YieldTimer
        //    Demo::YieldTimer yieldTimer( &timer );

        Ogre::uint64 startTime = timer.getMicroseconds();

        while( !mVideoLoader->getQuit() && !mGraphicsSystem->getQuit() )
        {
            mVideoLoader->update( );

            if( !renderWindow->isVisible() )
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
