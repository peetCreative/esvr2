#include "Esvr2OpenVRCompositorListener.h"

#include "Esvr2.h"
#include "Esvr2PoseState.h"

#include "OgreTextureGpuManager.h"
#include "OgreTextureGpu.h"
#include "OgreRenderSystem.h"
#include "OgreSceneNode.h"
#include "Compositor/Pass/OgreCompositorPass.h"
#include "Compositor/Pass/OgreCompositorPassDef.h"
#include "Compositor/Pass/PassScene/OgreCompositorPassScene.h"

#include "Compositor/OgreCompositorWorkspace.h"

namespace esvr2
{
    OpenVRCompositorListener::OpenVRCompositorListener(
            vr::IVRSystem *hmd, vr::IVRCompositor *vrCompositor,
            Ogre::TextureGpu *vrTexture, Ogre::Root *root,
            Ogre::CompositorWorkspace *workspaces[2],
            Ogre::SceneNode *camerasNode, std::shared_ptr<PoseState> poseState ) :
        mHMD( hmd ),
        mVrCompositor( vrCompositor ),
        mVrTexture( vrTexture ),
        mRoot( root ),
        mRenderSystem( root->getRenderSystem() ),
        mWorkspaces{ workspaces[LEFT], workspaces[RIGHT] },
        mCamerasNode( camerasNode ),
        mCameraPoseState( poseState ),
        mApiTextureType( vr::TextureType_Invalid ),
        mWaitingMode( VrWaitingMode::BeforeSceneGraph ),
        mFirstGlitchFreeMode( VrWaitingMode::NumVrWaitingModes ),
        mMustSyncAtEndOfFrame( false ),
        mFrameCnt(0),
        mTrackPose(false),
        mWriteTexture(false)
    {
        memset( mDevicePose, 0, sizeof( mDevicePose ) );

        mRoot->addFrameListener( this );
//      TODO: if we should consider using update poses properly
//         mWorkspaces[LEFT]->setListener( this );
//         if(mWorkspaces[RIGHT])
//             mWorkspaces[LEFT]->setListener( this );

        const Ogre::String &renderSystemName = mRenderSystem->getName();
        if( renderSystemName == "OpenGL 3+ Rendering Subsystem" )
            mApiTextureType = vr::TextureType_OpenGL;
    }

    //-------------------------------------------------------------------------
    OpenVRCompositorListener::~OpenVRCompositorListener()
    {
//         if( mWorkspaces[LEFT]->getListener() == this )
//             mWorkspaces[LEFT]->setListener( 0 );
//         if( mWorkspaces[RIGHT] && mWorkspaces[RIGHT]->getListener() == this )
//             mWorkspaces[RIGHT]->setListener( 0 );
        mRoot->removeFrameListener( this );
    }

    void OpenVRCompositorListener::syncCamera(void)
    {
        if (!mCamerasNode)
            LOG << "mCamerasNode does not exist" << LOGEND;
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
    }

    //-------------------------------------------------------------------------
    void OpenVRCompositorListener::updateHmdTrackingPose(void)
    {
//         if (mCameraPoseState->validPose())
            syncCamera();

        if( !mVrCompositor)
            return;
        // we have to call this so we get focus of the application
        mVrCompositor->WaitGetPoses(
            mTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );


//         if( !mTrackPose )
//             return;
//
//         for( size_t nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
//         {
//             if ( mTrackedDevicePose[nDevice].bPoseIsValid )
//             {
//                 mDevicePose[nDevice] = convertSteamVRMatrixToMatrix(
//                                            mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking );
//             }
//         }
//
//         if( mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
//         {
//             const bool canSync = canSyncCameraTransformImmediately();
//             if( canSync )
//                 syncCamera();
//             else
//                 mMustSyncAtEndOfFrame = true;
//         }
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
        if( mVrTexture->requiresTextureFlipping() )
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
            mVrTexture->writeContentsToFile("texture.bmp", 8, 255);
            mWriteTexture = false;
        }

        //TODO: not sure this is important
        Ogre::TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
        textureManager->waitForStreamingCompletion();

        mVrTexture->waitForData();
        mVrTexture->getCustomAttribute(
            Ogre::TextureGpu::msFinalTextureBuffer,
            &eyeTexture.handle );

        if (mHMD)
        {
            texBounds.uMin = 0;
            texBounds.uMax = 0.5f;
            mVrCompositor->Submit( vr::Eye_Left, &eyeTexture, &texBounds );
            texBounds.uMin = 0.5f;
            texBounds.uMax = 1.0f;
            mVrCompositor->Submit( vr::Eye_Right, &eyeTexture, &texBounds );
        }

        mRenderSystem->flushCommands();

        return true;
    }

    //-------------------------------------------------------------------------
    bool OpenVRCompositorListener::frameEnded( const Ogre::FrameEvent& evt )
    {
        if(mHMD)
        {
            mVrCompositor->PostPresentHandoff();
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
