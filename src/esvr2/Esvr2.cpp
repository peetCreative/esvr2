#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2NoneVideoLoader.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2PoseState.h"

#include "OgreTimer.h"

#include "Threading/OgreThreads.h"

#include <mutex>
#include <memory>
#include <boost/bind.hpp>

namespace esvr2 {

    struct ThreadParams {
        Esvr2* mEsvr2 {nullptr};
        boost::function<void(Ogre::uint64)> mUpdateFct;
        ThreadParams(Esvr2 *esvr2, boost::function<void(Ogre::uint64)> updateFct):
            mEsvr2(esvr2), mUpdateFct(updateFct) {};
    };
    typedef std::vector<ThreadParams*> ThreadParamsVec;

    //! \brief callback to update the loop of esvr2
    unsigned long updateThread(Ogre::ThreadHandle *threadHandle)
    {
        ThreadParams *tp =
                reinterpret_cast<ThreadParams*>( threadHandle->getUserParam() );
        return tp->mEsvr2->updateThread(tp->mUpdateFct);
    }

    THREAD_DECLARE( updateThread )

    //! \brief convert string to ControllerType
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

    //! \brief convert string to VideoInputType
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

    //! \brief convert string to InputType
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

    //! \brief convert string to VideoRenderTarget
    VideoRenderTarget getRenderVideoTarget(std::string input_str)
    {
        VideoRenderTarget input = VRT_TO_SQUARE;
        if (input_str == "TO_SQUARE")
            input = VRT_TO_SQUARE;
        if (input_str == "TO_2D_RECTANGLE")
            input = VRT_TO_2D_RECTANGLE;
        return input;
    }

    //! \brief convert string to WorkspaceType
    WorkspaceType getWorkspaceType(std::string workspace_str)
    {
        WorkspaceType workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str == "TWO_CAMERAS_STEREO" )
            workspace = WS_TWO_CAMERAS_STEREO;
        if( workspace_str == "INSTANCED_STEREO" )
            workspace = WS_INSTANCED_STEREO;
        return workspace;
    }

    //! \brief convert string to Distortion
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

    //! \brief constructor of the Esvr2 Object
    //! \param config fully configured Esvr2Config
    Esvr2::Esvr2(std::shared_ptr<Esvr2Config> config):
            mConfig(config)
    {
        mGraphicsSystem = std::make_shared<GraphicsSystem>(this);
    }

    //! \brief Deconstructor of the Esvr2 Object
    Esvr2::~Esvr2()
    {

    }

    //! \brief makes the Application stop
    void Esvr2::quit()
    {
        if (mGraphicsSystem)
            mGraphicsSystem->quit();
    }

    //! \brief Checks if any component is to be stopped
    bool Esvr2::getQuit()
    {
        return
            mGraphicsSystem->getQuit() ||
            mComponents.end() != find_if(
                    mComponents.begin(), mComponents.end(),
                    [](ComponentPtr c){return c->getQuit();});
    }

    //! \brief Sets the VideoLoader object if needed
    bool Esvr2::setVideoLoader(VideoLoaderPtr videoLoader)
    {
        if (mIsConfigured)
        {
            LOG_ERROR << "Called setLaparoscopeController() after run()" << LOGEND;
            return false;
        }
        mComponents.push_back(videoLoader);
        mVideoLoader = videoLoader;
        return true;
    }

    //! \brief Sets the LaparoscopeController object
    //! to make Robot or simulated camera move
    bool Esvr2::setLaparoscopeController(
            LaparoscopeControllerPtr laparoscopeController)
    {
        if (mIsConfigured)
        {
            LOG_ERROR << "Called setLaparoscopeController() after run()" << LOGEND;
            return false;
        }
        mComponents.push_back(laparoscopeController);
        mLaparoscopeController = laparoscopeController;
        return true;
    }

    //! \brief Sets the PoseState object
    //! to make the application know where the laparoscope is
    bool Esvr2::setPoseState(PoseStatePtr poseState)
    {
        if (mIsConfigured)
        {
            LOG_ERROR << "Called setPoseState() after run()" << LOGEND;
            return false;
        }
        mComponents.push_back(poseState);
        mPoseState = poseState;
        return true;
    }

    //! \brief Function to get the tracked head position
    /*! \param rotation Quaternion of the VR-Headset Orientation
     *  \param translation 3D-Vector of the VR-Headset Position
     *  \return success
     */
    bool Esvr2::getHeadPose(std::array<Real, 3> &translation, std::array<Real, 4> &rotation) {
        //TODO: make this thread safe
        if(mGraphicsSystem && mGraphicsSystem->isReady())
        {
            Ogre::Vector3 headPosition = mGraphicsSystem->getGameState()->getHeadPosition();
            Ogre::Quaternion headOrientation = mGraphicsSystem->getGameState()->getHeadOrientation();
            translation = {headPosition.x, headPosition.y, headPosition.z};
            rotation = {headOrientation.w, headOrientation.x, headOrientation.y, headOrientation.z};
            return true;
        }
        return false;
    }

    //! \brief Function to register more function, which are called in the update loop
    //! \return success
    bool Esvr2::registerUpdateCallback(
            const boost::function<void(uint64)> updateFunction)
    {
        mUpdateCallbacks.push_back(updateFunction);
        return true;
    }

    //! \brief function to start the VR-Application
    //! \return c-style success of application
    int Esvr2::run()
    {
        mIsConfigured = true;
        if (!mVideoLoader)
        {
            mVideoLoader = std::make_shared<NoneVideoLoader>();
            mComponents.push_back(mVideoLoader);
        }
        for(auto componentIt: mComponents)
        {
            if(!componentIt->initialize() )
            {
                LOG_ERROR << "Could not initialize a Component" << LOGEND;
                return 1;
            }
        }
        // cycle until videoLoader is finished or quits
        bool allComponentsReady {false};
        Ogre::Timer timer;
        while (!allComponentsReady){
            for(auto fctIt: mUpdateCallbacks)
            {
                (fctIt)(timer.getMicroseconds());
            }
            allComponentsReady = mComponents.end() == std::find_if(
                    mComponents.begin(), mComponents.end(),
                    [](ComponentPtr a){return !a->isReady();});
            if (getQuit())
            {
                return 0;
            }
        }
        mGraphicsSystem->initialize();
        mComponents.push_back(mGraphicsSystem);
        registerUpdateCallback(
                boost::bind(&GraphicsSystem::update, mGraphicsSystem, _1));
        if ( mConfig->multithreading )
        {
            LOG << "multiThreading" << LOGEND;
            //because ogre we need to keep track the memory the old way
            ThreadParamsVec threadParams;
            Ogre::ThreadHandleVec mThreadHandles;
            int i = 0;
            for (auto updateCallback: mUpdateCallbacks)
            {
                ThreadParams *params = new ThreadParams(this, updateCallback);
                threadParams.push_back(params);
                mThreadHandles.push_back(Ogre::Threads::CreateThread(
                        THREAD_GET( updateThread ), i++, params));
            }
            Ogre::Threads::WaitForThreads( mThreadHandles );
            for (auto params: threadParams)
            {
                delete params;
            }
        }
            //SINGLETHREADED
        else
        {
            LOG << "singleThreading" << LOGEND;
            while( !getQuit() )
            {
                for(auto fctIt: mUpdateCallbacks)
                {
                    (fctIt)(timer.getMicroseconds());
                }
                mGraphicsSystem->update( timer.getMicroseconds() );

                if( !mGraphicsSystem->isRenderWindowVisible() )
                {
                    //Don't burn CPU cycles unnecessary when we're minimized.
                    Ogre::Threads::Sleep( 500 );
                }
            }
            LOG << "END GRAPHICS" << LOGEND;

        }
        //This is not doing any thing
        mGraphicsSystem->deinitialize();
        for (auto component : mComponents)
            component->deinitialize();
        return 0;
    }

    //---------------------------------------------------------------------
    unsigned long Esvr2::updateThread(
            boost::function<void(Ogre::uint64)> updateFct)
    {
        Ogre::Timer timer;

        while( !getQuit() )
        {
            updateFct(timer.getMicroseconds());
            if( !mGraphicsSystem->isRenderWindowVisible() )
            {
                //Don't burn CPU cycles unnecessary when we're minimized.
                Ogre::Threads::Sleep( 500 );
            }
        }
        return 0;
    };
}
