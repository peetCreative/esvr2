#include "StereoRenderingGraphicsSystem.h"

#include "StereoRendering.h"
#include "OpenVRCompositorListener.h"

#include "OgreTextureGpuManager.h"
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
// #include "Compositor/OgreCompositorManager2.h"

using namespace Demo;

namespace Demo
{
    //-------------------------------------------------------------------------------
    void StereoGraphicsSystem::createCamera(void)
    {
        //Use one node to control both cameras
        mCamerasNode = mSceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mCamerasNode->setName( "Cameras Node" );

        mCamerasNode->setPosition( 0, 0, 0 );
        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {

            mEyeCameras[LEFT] = mSceneManager->createCamera( "Left Eye" );
            mEyeCameras[RIGHT] = mSceneManager->createCamera( "Right Eye" );

            const Ogre::Real eyeDistance        = 0.06f;
            const Ogre::Real eyeFocusDistance   = 0.06f;

            for( int leftOrRight = 0; leftOrRight < 2; ++leftOrRight )
            {
                const Ogre::Vector3 camPos( eyeDistance * (leftOrRight * 2 - 1), 0, 0 );
                mEyeCameras[leftOrRight]->setPosition( camPos );

                Ogre::Vector3 lookAt( eyeFocusDistance * (leftOrRight * 2 - 1), 0, -1 );
                //Ogre::Vector3 lookAt( 0, 0, 0 );

                // Look back along -Z
                mEyeCameras[leftOrRight]->lookAt( lookAt );
                mEyeCameras[leftOrRight]->setNearClipDistance( 0.2f );
                mEyeCameras[leftOrRight]->setFarClipDistance( 1000.0f );
                mEyeCameras[leftOrRight]->setAutoAspectRatio( true );

                //By default cameras are attached to the Root Scene Node.
                mEyeCameras[leftOrRight]->detachFromParent();
                mCamerasNode->attachObject( mEyeCameras[leftOrRight] );
            }

            mCamera = mEyeCameras[LEFT];
        }
        if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mCamera = mSceneManager->createCamera( "Main Camera" );

            // Position it at 500 in Z direction
            mCamera->setPosition( Ogre::Vector3( 0.0, 0.0, 0.0 ) );
            // Look back along -Z
            mCamera->lookAt( Ogre::Vector3( 0, 0, -1.0 ) );
            mCamera->setNearClipDistance( 0.2f );
            mCamera->setFarClipDistance( 1000.0f );
            mCamera->setAutoAspectRatio( true );
            mCamera->detachFromParent();
            mCamerasNode->attachObject( mCamera );
            mEyeCameras[LEFT] = mCamera;
        }

    }

    Ogre::CompositorWorkspace* StereoGraphicsSystem::setupCompositor(void)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        mVrCullCamera = mSceneManager->createCamera( "VrCullCamera" );
        initOpenVR();

        const Ogre::IdString workspaceName( "StereoMirrorWindowWorkspace" );

        Ogre::CompositorChannelVec channels( 2u );
        channels[0] = mRenderWindow->getTexture();
        channels[1] = mVrTexture;
        mMirrorWorkspace = compositorManager->addWorkspace(
            mSceneManager, channels, mCamera,
            workspaceName, true );


        return mMirrorWorkspace;
    }

    //-----------------------------------------------------------------------------
    // Purpose: Helper to get a string from a tracked device property and turn it
    //			into a std::string
    //-----------------------------------------------------------------------------
    std::string StereoGraphicsSystem::GetTrackedDeviceString(
        vr::TrackedDeviceIndex_t unDevice,
        vr::TrackedDeviceProperty prop,
        vr::TrackedPropertyError *peError)
    {
        vr::IVRSystem *vrSystem = vr::VRSystem();
        uint32_t unRequiredBufferLen =
            vrSystem->GetStringTrackedDeviceProperty(
                unDevice, prop,
                NULL, 0, peError );
        if( unRequiredBufferLen == 0 )
            return "";

        char *pchBuffer = new char[ unRequiredBufferLen ];
        unRequiredBufferLen = vrSystem->GetStringTrackedDeviceProperty(
            unDevice, prop, pchBuffer,
            unRequiredBufferLen, peError );
        std::string sResult = pchBuffer;
        delete [] pchBuffer;
        return sResult;
    }

    void StereoGraphicsSystem::initCompositorVR(void)
    {
        mVRCompositor = vr::VRCompositor();
        if ( !mVRCompositor )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_RENDERINGAPI_ERROR,
                         "VR Compositor initialization failed. See log file for details",
                         "StereoRenderingGraphicsSystem::initCompositorVR" );
        }
    }

    void StereoGraphicsSystem::initOpenVR(void)
    {
        // Loading the SteamVR via OpenVR Runtime
        LOG << "initOpenVR" << LOGEND;
        if(vr::VR_IsHmdPresent())
        {
            vr::EVRInitError eError = vr::VRInitError_None;
            mHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
            if( eError != vr::VRInitError_None )
            {
                mHMD = nullptr;
                LOG << "OpenVR not even if HMD seems to be present. Have you started SteamVR?" << LOGEND;
            }
        }

        uint32_t width, height, new_width;
        if ( mHMD )
        {
            mStrDriver = "No Driver";
            mStrDisplay = "No Display";

            mStrDriver = GetTrackedDeviceString(
                vr::k_unTrackedDeviceIndex_Hmd,
                vr::Prop_TrackingSystemName_String );
            mStrDisplay = GetTrackedDeviceString(
                vr::k_unTrackedDeviceIndex_Hmd,
                vr::Prop_SerialNumber_String );
            mDeviceModelNumber = GetTrackedDeviceString(
                vr::k_unTrackedDeviceIndex_Hmd,
                vr::Prop_ModelNumber_String );

            initCompositorVR();

            //gives us the render target off one eye
            mHMD->GetRecommendedRenderTargetSize( &width, &height );
        }
        else
        {
            width = mRenderWindow->getWidth();
            height = mRenderWindow->getHeight();
        }

        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
        //Radial Density Mask requires the VR texture to be UAV & reinterpretable
        mVrTexture = textureManager->createOrRetrieveTexture(
            "OpenVR Both Eyes",
            Ogre::GpuPageOutStrategy::Discard,
            Ogre::TextureFlags::RenderToTexture|
            Ogre::TextureFlags::Uav|
            Ogre::TextureFlags::Reinterpretable,
            Ogre::TextureTypes::Type2D );
        new_width = width << 1u;
        mVrTexture->setResolution( new_width, height);
        mVrTexture->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
        mVrTexture->scheduleTransitionTo(
            Ogre::GpuResidency::Resident );

        Ogre::CompositorManager2 *compositorManager =
            mRoot->getCompositorManager2();

        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {
            const Ogre::IdString workspaceName( "TwoCamerasWorkspace" );
            Ogre::uint8 vpModifierMask, executionMask;
            Ogre::Vector4 vpOffsetScale;

            vpModifierMask  = 0x01;
            executionMask   = 0x01;
            vpOffsetScale   = Ogre::Vector4( 0.0f, 0.0f, 0.5f, 1.0f );
            mVrWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager,
                mRenderWindow->getTexture(),
                mEyeCameras[LEFT], workspaceName,
                true, -1, (Ogre::UavBufferPackedVec*)0,
                (Ogre::ResourceLayoutMap*)0,
                (Ogre::ResourceAccessMap*)0,
                vpOffsetScale,
                vpModifierMask,
                executionMask );

            vpModifierMask  = 0x02;
            executionMask   = 0x02;
            vpOffsetScale   = Ogre::Vector4( 0.5f, 0.0f, 0.5f, 1.0f );
            mVrWorkspaces[RIGHT] = compositorManager->addWorkspace(
                mSceneManager,
                mRenderWindow->getTexture(),
                mEyeCameras[RIGHT], workspaceName,
                true, -1, (Ogre::UavBufferPackedVec*)0,
                (Ogre::ResourceLayoutMap*)0,
                (Ogre::ResourceAccessMap*)0,
                vpOffsetScale,
                vpModifierMask,
                executionMask);
        }
        else if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mVrWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager, mVrTexture,
                mCamera, "InstancedStereoWorkspace", true, 0 );
        }

        int frames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();
        HmdConfig hmdConfig{
            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
            { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY },
            { {-1.3,1.3,-1.45,1.45}, {-1.3,1.3,-1.45,1.45} }
        };

        mOvrCompositorListener =
            new Demo::OpenVRCompositorListener(
                mHMD, mVRCompositor, mVrTexture,
                mRoot, mVrWorkspaces,
                mEyeCameras, mVrCullCamera,
                frames, hmdConfig );
    }

    StereoGraphicsSystem::StereoGraphicsSystem( GameState* gameState, WorkspaceType wsType ) :
        GraphicsSystem( gameState, "../Data/" ),
        mWorkSpaceType( wsType ),
        mCamerasNode( nullptr ),
        mEyeCameras{ nullptr, nullptr },
        mVrWorkspaces{ nullptr, nullptr },
        mMirrorWorkspace( nullptr ),
        mVrCullCamera( nullptr ),
        mVrTexture( nullptr ),
        mOvrCompositorListener( nullptr ),
        mHMD( nullptr ),
        mVRCompositor( nullptr ),
        mStrDriver( "" ),
        mStrDisplay( "" ),
        mDeviceModelNumber( "" )
    {
        memset( mTrackedDevicePose, 0, sizeof (mTrackedDevicePose) );
    }

    StereoGraphicsSystem::~StereoGraphicsSystem()
    {
    //             delete mEyeCameras;
    }
}
