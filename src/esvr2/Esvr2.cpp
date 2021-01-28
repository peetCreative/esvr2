#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2LowLatencyVideoLoader.h"
#include "Esvr2TestPose.h"
#include "Esvr2Barrier.h"

#include "OgreTimer.h"

#include "Threading/OgreThreads.h"

#include <mutex>
#include <memory>

namespace esvr2 {
    extern const double cFrametime;
    const double cFrametime = 1.0 / 25.0;

    unsigned long renderThread(Ogre::ThreadHandle *threadHandle)
    {
        Esvr2 *esvr2 = reinterpret_cast<Esvr2*>( threadHandle->getUserParam() );
        return esvr2->renderThread1();
    };

    unsigned long logicThread(Ogre::ThreadHandle *threadHandle)
    {
        Esvr2 *esvr2 = reinterpret_cast<Esvr2*>( threadHandle->getUserParam() );
        return esvr2->logicThread1();
    };

    THREAD_DECLARE( renderThread );
    THREAD_DECLARE( logicThread );

    ControllerType getControllerType(std::string input_str)
    {
        ControllerType controllerType = CT_NONE;
        if (input_str == "OPT0")
            controllerType = CT_OPT0;
        if (input_str == "OPT1")
            controllerType = CT_OPT1;
        if (input_str == "OPT2")
            controllerType = CT_OPT2;
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
            Esvr2Config *config,
            VideoLoader *videoLoader,
            LaparoscopeController *laparoscopeController,
            PoseState *poseState)
    {
        std::shared_ptr<Esvr2Config> sConfig(config);
        std::shared_ptr<VideoLoader> sVideoLoader(videoLoader);
        std::shared_ptr<LaparoscopeController> sLaparoscopeController(laparoscopeController);
        std::shared_ptr<PoseState> sPoseState(poseState);
        Esvr2(sConfig, sVideoLoader,
              sLaparoscopeController,
              sPoseState );
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
            mBarrier(new Barrier())
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
            mVideoLoader->update();
            if (mVideoLoader->getQuit())
            {
                LOG << "quit videoLoader" << LOGEND;
                return;
            }
        }
        if (mLaparoscopeController)
        {
            while( !mLaparoscopeController->isReady() )
            {
                Ogre::Threads::Sleep( 50 );
                mLaparoscopeController->update();
                if (mLaparoscopeController->getQuit())
                {
                    LOG << "could not get laparoscopeController ready" << LOGEND;
                    return;
                }
            }
        }


        mGraphicsSystem = std::make_shared<GraphicsSystem>( this );

        mGraphicsSystem->initialize();

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
                mGraphicsSystem->update( startTime );

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
            mGraphicsSystem->deinitialize();
            mVideoLoader->deinitialize();
        }
        return 0;
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

        mBarrier->sync();

        mGraphicsSystem->deinitialize();
        mBarrier->sync();

        return 0;
    };

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

        mBarrier->sync();

        mVideoLoader->deinitialize();
        mBarrier->sync();

        return 0;
    };
}
