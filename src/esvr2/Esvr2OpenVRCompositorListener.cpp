#include "Esvr2OpenVRCompositorListener.h"

#include <utility>

#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2PoseState.h"

#include "OgreTextureGpuManager.h"
#include "OgreTextureGpu.h"
#include "Compositor/Pass/OgreCompositorPass.h"
#include "Compositor/Pass/OgreCompositorPassDef.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassScene.h"
#include "Vao/OgreVaoManager.h"


#include "Compositor/OgreCompositorWorkspace.h"

namespace esvr2
{
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

    OpenVRCompositorListener::OpenVRCompositorListener(
            GraphicsSystem *graphicsSystem ) :
        mGraphicsSystem(graphicsSystem),
        mRenderSystem(graphicsSystem->mRoot->getRenderSystem()),
        mApiTextureType( vr::TextureType_Invalid ),
        mWaitingMode( VrWaitingMode::BeforeSceneGraph ),
        mFirstGlitchFreeMode( VrWaitingMode::NumVrWaitingModes ),
        mMustSyncAtEndOfFrame( false ),
        mTrackPose(true),
        mValidPoseCount(0),
        mWriteTexture(false)
    {
        memset( mTrackedDevicePose, 0, sizeof( mTrackedDevicePose ) );
        memset( mDevicePose, 0, sizeof( mDevicePose ) );

        mApiTextureType = vr::TextureType_OpenGL;
    }

    //-------------------------------------------------------------------------
    OpenVRCompositorListener::~OpenVRCompositorListener()
    {
//        mGraphicsSystem->mVRWorkspaces[RIGHT]->removeListener(this);
        mGraphicsSystem->mRoot->removeFrameListener( this );
        if( mHMD )
        {
            vr::VR_Shutdown();
            mHMD = NULL;
        }
    }

    //-----------------------------------------------------------------------------
    // Purpose: Helper to get a string from a tracked device property and turn it
    //			into a std::string
    //-----------------------------------------------------------------------------
    std::string OpenVRCompositorListener::GetTrackedDeviceString(
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

    bool OpenVRCompositorListener::initOpenVR(void)
    {
        // Loading the SteamVR via OpenVR Runtime
        LOG << "initOpenVR" << LOGEND;
        if(vr::VR_IsHmdPresent())
        {
            vr::EVRInitError eError = vr::VRInitError_None;
            mHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
            if( eError != vr::VRInitError_None )
            {
                LOG << "VR HMD initialization failed."
                    << "See log file for details" << LOGEND;
                return false;
            }
        }
        else
        {
            return false;
        }

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

            mVRCompositor = vr::VRCompositor();
            if ( !mVRCompositor )
            {
                 LOG << "VR Compositor initialization failed."
                     << " See log file for details" << LOGEND;
                 return false;
            }
        }
        return true;
    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::updateHmdTrackingPose(void)
    {
        if( !mVRCompositor)
            return;
        // we have to call this so we get focus of the application
        mVRCompositor->WaitGetPoses(
            mTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );


        if( !mTrackPose )
            return;

        for( size_t nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
        {
            if ( mTrackedDevicePose[nDevice].bPoseIsValid )
            {
                mDevicePose[nDevice] = convertSteamVRMatrixToMatrix(
                                           mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
            }
        }

        if( mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
        {
            const bool canSync = canSyncCameraTransformImmediately();
            if( canSync )
                syncCamera();
            else
                mMustSyncAtEndOfFrame = true;
        }
    }
    //We do not need because we move the whole Node
//    void OpenVRCompositorListener::syncCullCamera(void)
//    {
//        const Ogre::Quaternion derivedRot = mCamera->getDerivedOrientation();
//        Ogre::Vector3 camPos = mCamera->getDerivedPosition();
//        mVrCullCamera->setOrientation( derivedRot );
//        mVrCullCamera->setPosition( camPos + derivedRot * mCullCameraOffset );
//    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncCamera(void)
    {
        if (mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
        {
            OGRE_ASSERT_MEDIUM( mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid );
            Ogre::Vector3 trans = mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].getTrans();
            mGraphicsSystem->mGameState->mVRCamerasNode->setPosition( trans );
            Ogre::Quaternion orientation = mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].extractQuaternion();
            mGraphicsSystem->mGameState->mVRCamerasNode->setOrientation( orientation );
        }

//        if( mWaitingMode < VrWaitingMode::AfterFrustumCulling )
//            syncCullCamera();

        mMustSyncAtEndOfFrame = false;
    }

    inline void printMatrix4(Ogre::Matrix4 m)
    {
        LOG << m[0][0] << " "
            << m[0][1] << " "
            << m[0][2] << " "
            << m[0][3] << " " << LOGEND
            << m[1][0] << " "
            << m[1][1] << " "
            << m[1][2] << " "
            << m[1][3] << " " << LOGEND
            << m[2][0] << " "
            << m[2][1] << " "
            << m[2][2] << " "
            << m[2][3] << " " << LOGEND
            << m[3][0] << " "
            << m[3][1] << " "
            << m[3][2] << " "
            << m[3][3] << " " << LOGEND;

    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::syncVRCameraProjection( bool bForceUpdate )
    {
        if( bForceUpdate )
        {
            Ogre::Matrix4 eyeToHead[2];
            Ogre::Matrix4 projectionMatrix[2];
            Ogre::Matrix4 projectionMatrixRS[2];
            Ogre::Vector4 eyeFrustumExtents[2];

            for( size_t i=0u; i<2u; ++i )
            {
                vr::EVREye eyeIdx = static_cast<vr::EVREye>( i );
                eyeToHead[i] = convertSteamVRMatrixToMatrix(
                        mHMD->GetEyeToHeadTransform( eyeIdx ) );
                projectionMatrix[i] =
                        convertSteamVRMatrixToMatrix(
                                mHMD->GetProjectionMatrix(
                                        eyeIdx,
                                        mGraphicsSystem->mVRCameraNear,
                                        mGraphicsSystem->mVRCameraFar ) );
                mHMD->GetProjectionRaw(
                        eyeIdx,
                        &eyeFrustumExtents[i].x, &eyeFrustumExtents[i].y,
                        &eyeFrustumExtents[i].z, &eyeFrustumExtents[i].w );

                mGraphicsSystem->mRoot->getRenderSystem()
                    ->_convertOpenVrProjectionMatrix(
                        projectionMatrix[i], projectionMatrixRS[i] );

                LOG<< "eyeToHead"<< LOGEND;
                printMatrix4(eyeToHead[i]);
                LOG<< "projectionMatrix"<< LOGEND;
                printMatrix4(projectionMatrix[i]);
                LOG<< "projectionMatrixRS"<< LOGEND;
                printMatrix4(projectionMatrixRS[i]);

                mGraphicsSystem->mVRCameras[i]->setCustomProjectionMatrix(
                    true, projectionMatrixRS[i]);
                mGraphicsSystem->mVRCameras[i]->setPosition(
                        eyeToHead[i].getTrans());
            }
//TODO: maybe we can also use instanced stereo

// mVrData.set( eyeToHead, projectionMatrixRS );
//            if (mGraphicsSystem->)
//            mGraphicsSystem->mVrData.set( eyeToHead, projectionMatrixRS );

            Ogre::Vector4 cameraCullFrustumExtents;
            cameraCullFrustumExtents.x = std::min(
                    eyeFrustumExtents[0].x, eyeFrustumExtents[1].x );
            cameraCullFrustumExtents.y = std::max(
                    eyeFrustumExtents[0].y, eyeFrustumExtents[1].y );
            cameraCullFrustumExtents.z = std::max(
                    eyeFrustumExtents[0].z, eyeFrustumExtents[1].z );
            cameraCullFrustumExtents.w = std::min(
                    eyeFrustumExtents[0].w, eyeFrustumExtents[1].w );

            mGraphicsSystem->mVRCullCamera->setFrustumExtents(
                    cameraCullFrustumExtents.x,
                    cameraCullFrustumExtents.y,
                    cameraCullFrustumExtents.w,
                    cameraCullFrustumExtents.z,
                    Ogre::FET_TAN_HALF_ANGLES );

            const float ipd = mGraphicsSystem->mVrData.mLeftToRight.x;
            Ogre::Vector3 cullCameraOffset = Ogre::Vector3::ZERO;
            cullCameraOffset.z = (ipd / 2.0f) /
                                 Ogre::Math::Abs( cameraCullFrustumExtents.x );

            const Ogre::Real offset = cullCameraOffset.length();
            mGraphicsSystem->mVRCullCamera->setNearClipDistance(
                    mGraphicsSystem->mVRCameraNear + offset );
            mGraphicsSystem->mVRCullCamera->setFarClipDistance(
                    mGraphicsSystem->mVRCameraFar + offset );
            mGraphicsSystem->mVRCullCamera->lookAt(0,0,-1);
        }
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameStarted( const Ogre::FrameEvent& evt )
    {
        if( mWaitingMode == VrWaitingMode::BeforeSceneGraph )
            updateHmdTrackingPose();

        return true;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameRenderingQueued( const Ogre::FrameEvent &evt )
    {
        vr::VRTextureBounds_t texBounds;
        if( mGraphicsSystem->mVRTexture->requiresTextureFlipping() )
        {
            texBounds.vMin = 1.0f;
            texBounds.vMax = 0.0f;
        }
        else
        {
            texBounds.vMin = 0.0f;
            texBounds.vMax = 1.0f;
        }

        vr::Texture_t eyeTexture =
        {
            0,
            mApiTextureType,
            vr::ColorSpace_Gamma
        };
        if (mWriteTexture)
        {
            mGraphicsSystem->mVRTexture->writeContentsToFile("texture.bmp", 8, 255);
            mWriteTexture = false;
        }

        //TODO: not sure this is important
        Ogre::TextureGpuManager *textureManager =
            mRenderSystem->getTextureGpuManager();
        textureManager->waitForStreamingCompletion();

        mGraphicsSystem->mVRTexture->waitForData();
        mGraphicsSystem->mVRTexture->getCustomAttribute(
            Ogre::TextureGpu::msFinalTextureBuffer,
            &eyeTexture.handle );

        if (mHMD)
        {
            texBounds.uMin = 0;
            texBounds.uMax = 0.5f;
            mVRCompositor->Submit( vr::Eye_Left, &eyeTexture, &texBounds );
            texBounds.uMin = 0.5f;
            texBounds.uMax = 1.0f;
            mVRCompositor->Submit( vr::Eye_Right, &eyeTexture, &texBounds );
        }

        mRenderSystem->flushCommands();

        if (mHMD)
        {

            vr::VREvent_t event;
            while( mHMD->PollNextEvent( &event, sizeof(event) ) )
            {
                if( event.trackedDeviceIndex != vr::k_unTrackedDeviceIndex_Hmd &&
                    event.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid )
                {
                    continue;
                }

                switch( event.eventType )
                {
                    case vr::VREvent_TrackedDeviceUpdated:
                    case vr::VREvent_IpdChanged:
                    case vr::VREvent_ChaperoneDataHasChanged:
                        syncVRCameraProjection( true );
                        break;
                }
            }
        }

        return true;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt ) {
        //TODO: do we need this here
        syncVRCameraProjection(false);
        //TODO: do we need to gueard this
        if (mHMD) {
            if (mWaitingMode == VrWaitingMode::AfterSwap)
                updateHmdTrackingPose();
            else
                mVRCompositor->PostPresentHandoff();
            if (mMustSyncAtEndOfFrame)
                syncCamera();
//            if (mWaitingMode >= VrWaitingMode::AfterFrustumCulling)
//                syncCullCamera();
            mVRCompositor->PostPresentHandoff();
        }
        return true;
    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::workspacePreUpdate( Ogre::CompositorWorkspace *workspace )
    {
        if( mWaitingMode == VrWaitingMode::AfterSceneGraph )
            updateHmdTrackingPose();
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passPreExecute( Ogre::CompositorPass *pass )
    {
        if( mWaitingMode == VrWaitingMode::BeforeShadowmaps &&
            pass->getDefinition()->getType() == Ogre::PASS_SCENE &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass )
    {
        if( mWaitingMode == VrWaitingMode::BeforeFrustumCulling &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass )
    {
        if( mWaitingMode == VrWaitingMode::AfterFrustumCulling &&
            pass->getDefinition()->mIdentifier == 0x01234567 )
        {
            updateHmdTrackingPose();
        }
    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::setWaitingMode( VrWaitingMode::VrWaitingMode waitingMode )
    {
        mWaitingMode = waitingMode;
    }
    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::setGlitchFree( VrWaitingMode::VrWaitingMode firstGlitchFreeMode )
    {
        mFirstGlitchFreeMode = firstGlitchFreeMode;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::canSyncCameraTransformImmediately(void) const
    {
        return mWaitingMode <= VrWaitingMode::BeforeSceneGraph ||
               mWaitingMode <= mFirstGlitchFreeMode;
    }
}
