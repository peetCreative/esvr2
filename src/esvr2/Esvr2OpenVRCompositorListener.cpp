#include "Esvr2OpenVRCompositorListener.h"

#include <utility>

#include "Esvr2.h"
#include "Esvr2PoseState.h"

#include "OgreTextureGpuManager.h"
#include "OgreTextureGpu.h"
#include "OgreSceneNode.h"
#include "Compositor/Pass/OgreCompositorPass.h"
#include "Compositor/Pass/OgreCompositorPassDef.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassScene.h"

#include "Compositor/OgreCompositorWorkspace.h"

namespace esvr2
{
    OpenVRCompositorListener::OpenVRCompositorListener(
            GraphicsSystem *graphicsSystem,
            std::shared_ptr<PoseState> poseState ) :
        mGraphicsSystem(graphicsSystem),
        mRenderSystem(graphicsSystem->mRoot->getRenderSystem()),
        mCameraPoseState(std::move( poseState )),
        mApiTextureType( vr::TextureType_Invalid ),
        mWaitingMode( VrWaitingMode::BeforeSceneGraph ),
        mFirstGlitchFreeMode( VrWaitingMode::NumVrWaitingModes ),
        mMustSyncAtEndOfFrame( false ),
        mFrameCnt(0),
        mTrackPose(false),
        mValidPoseCount(0),
        mWriteTexture(false)
    {
        memset( mTrackedDevicePose, 0, sizeof( mTrackedDevicePose ) );
        memset( mDevicePose, 0, sizeof( mDevicePose ) );
        memset( &mVrData, 0, sizeof( mVrData ) );

        if ( mGraphicsSystem->mIsStereo )
        {
            mGraphicsSystem->mVRCameras[MONO]->setVrData(&mVrData);
        }

        mGraphicsSystem->mRoot->addFrameListener( this );
//      TODO: if we should consider using update poses properly
        mGraphicsSystem->mVRWorkspaces[LEFT]->addListener( this );
        if(mGraphicsSystem->mVRWorkspaces[RIGHT])
            mGraphicsSystem->mVRWorkspaces[RIGHT]->addListener( this );

        mApiTextureType = vr::TextureType_OpenGL;
    }

    //-------------------------------------------------------------------------
    OpenVRCompositorListener::~OpenVRCompositorListener()
    {
         if( mGraphicsSystem->mVRWorkspaces[LEFT] &&
                 mGraphicsSystem->mVRWorkspaces[LEFT]->getListener() == this )
             mGraphicsSystem->mVRWorkspaces[LEFT]->setListener( 0 );
         if( mGraphicsSystem->mVRWorkspaces[RIGHT] &&
                 mGraphicsSystem->mVRWorkspaces[RIGHT]->getListener() == this )
             mGraphicsSystem->mVRWorkspaces[RIGHT]->setListener( 0 );
        mGraphicsSystem->mRoot->removeFrameListener( this );
    }

//    void OpenVRCompositorListener::syncCamera(void)
//    {
//        if (!mGraphicsSystem->mVRCamerasNode)
//            LOG << "mCamerasNode does not exist" << LOGEND;
//         : take pose from mTrackedDevicePose
//         mTrackedDevicePose();

//         Ogre::Vector3 pos(
//             -0.0021829536705136577,
//             -0.04232839038721293,
//             0.14707343904246845);
//         Ogre::Quaternion rot(
//             0.9638096591466656,
//             -0.23380399717986433,
//             0.03972169619117772,
//             0.12177363708948678
//             );
//         rot = Ogre::Quaternion::IDENTITY;/*(Ogre::Degree(-45), Ogre::Vector3::UNIT_X);*/
//         mCamerasNode->setPosition( pos );
//         mCamerasNode->lookAt( Ogre::Vector3(0,0,0), Ogre::Node::TS_PARENT );
//         mCamerasNode->setPosition( mCameraPoseState->getPosition());
//         mCamerasNode->setOrientation( mCameraPoseState->getRotation());
//    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::updateHmdTrackingPose(void)
    {
        if (mCameraPoseState->validPose())
            syncCamera();
        if( !mGraphicsSystem->mVRCompositor)
            return;
        // we have to call this so we get focus of the application
        mGraphicsSystem->mVRCompositor->WaitGetPoses(
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
            mGraphicsSystem->mVRCamerasNode->setPosition( mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].getTrans() );
            mGraphicsSystem->mVRCamerasNode->setOrientation( mDevicePose[vr::k_unTrackedDeviceIndex_Hmd].extractQuaternion() );
        }

//        if( mWaitingMode < VrWaitingMode::AfterFrustumCulling )
//            syncCullCamera();

        mMustSyncAtEndOfFrame = false;
    }


    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameStarted( const Ogre::FrameEvent& evt )
    {
        if( mWaitingMode == VrWaitingMode::BeforeSceneGraph )
            updateHmdTrackingPose();
        mFrameCnt++;

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

        if (mGraphicsSystem->mHMD)
        {
            texBounds.uMin = 0;
            texBounds.uMax = 0.5f;
            mGraphicsSystem->mVRCompositor->Submit( vr::Eye_Left, &eyeTexture, &texBounds );
            texBounds.uMin = 0.5f;
            texBounds.uMax = 1.0f;
            mGraphicsSystem->mVRCompositor->Submit( vr::Eye_Right, &eyeTexture, &texBounds );
        }

        mRenderSystem->flushCommands();

        if (mGraphicsSystem->mHMD)
        {

            vr::VREvent_t event;
            while( mGraphicsSystem->mHMD->PollNextEvent( &event, sizeof(event) ) )
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
                        mGraphicsSystem->syncVRCameraProjection( true );
                        break;
                }
            }
        }

        return true;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt ) {
        mGraphicsSystem->syncVRCameraProjection(false);
        //TODO: do we need to gueard this
        if (mGraphicsSystem->mHMD) {
            if (mWaitingMode == VrWaitingMode::AfterSwap)
                updateHmdTrackingPose();
            else
                mGraphicsSystem->mVRCompositor->PostPresentHandoff();
            if (mMustSyncAtEndOfFrame)
                syncCamera();
//            if (mWaitingMode >= VrWaitingMode::AfterFrustumCulling)
//                syncCullCamera();
            mGraphicsSystem->mVRCompositor->PostPresentHandoff();
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
