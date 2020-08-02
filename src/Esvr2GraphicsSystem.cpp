#include "Esvr2GraphicsSystem.h"

#include "Esvr2StereoRendering.h"
#include "Esvr2OpenVRCompositorListener.h"

#include "GameState.h"

#include "OgrePlatform.h"
#include "OgreTextureGpuManager.h"
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
#include "Compositor/OgreCompositorNode.h"
#include "Compositor/OgreCompositorWorkspace.h"

#include "OgreTextureGpuManager.h"
#include "OgrePixelFormatGpuUtils.h"
#include "OgreMemoryAllocatorConfig.h"
#include "OgreGpuResource.h"
#include "OgreStagingTexture.h"
#include "Vao/OgreVaoManager.h"

#include "opencv2/opencv.hpp"
#include <sstream>
#include <cmath>
#include <mutex>

#include "SdlInputHandler.h"
#include <SDL_syswm.h>
#include <SDL.h>

namespace esvr2
{

    //-------------------------------------------------------------------------------
    void GraphicsSystem::createCamera(void)
    {
        //Use one node to control both cameras
//         mCamerasNode->setOrientation( orientation );
        mCamerasNode = mSceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mCamerasNode->setName( "Cameras Node" );
//         mCamerasNode->setPosition( pos );
//         mCamerasNode->setOrientation( orientation );

        mCameraNode[LEFT] = mCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mCameraNode[LEFT]->setName( "Left Camera Node" );

        mCameraNode[RIGHT] = mCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mCameraNode[RIGHT]->setName( "Right Camera Node" );

        mVrCullCamera = mSceneManager->createCamera( "VrCullCamera" );
//         mVrCullCamera->detachFromParent();
//         mCamerasNode->attachObject( mVrCullCamera );

//         mCamerasNode->setPosition(pos);

        //setup mVrData und mVrCullCamera using the mHmdConfig
        syncCameraProjection( true );

        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {
            mEyeCameras[LEFT] = mSceneManager->createCamera( "Left Eye" );
            mEyeCameras[RIGHT] = mSceneManager->createCamera( "Right Eye" );

//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;

            //By default cameras are attached to the Root Scene Node.
            mEyeCameras[LEFT]->detachFromParent();
            mCamerasNode->attachObject( mEyeCameras[LEFT] );
            mEyeCameras[RIGHT]->detachFromParent();
            mCamerasNode->attachObject( mEyeCameras[RIGHT] );
//             mCameraNode[RIGHT]->attachObject( mEyeCameras[RIGHT] );

            mCamera = mEyeCameras[RIGHT];
        }
        if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mCamera = mSceneManager->createCamera( "Main Camera" );

            // Position it at 500 in Z direction
            mCamera->setPosition( Ogre::Vector3( 0.0, 0.0, 0.0 ) );
            // Look back along -Z
            mCamera->lookAt( Ogre::Vector3( 0, 0, -1.0 ) );
            mCamera->setNearClipDistance( mCamNear );
            mCamera->setFarClipDistance( mCamFar );
            mCamera->setAutoAspectRatio( true );
            mCamera->detachFromParent();
            mCamera->setVrData( mVrData );
            mCamerasNode->attachObject( mCamera );
            mEyeCameras[LEFT] = mCamera;
        }
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

    void GraphicsSystem::alignCameras()
    {
        //                 const Ogre::Vector3 camPos( eyeDistance * (leftOrRight * 2 - 1), 0, 0 );


    }
    //-------------------------------------------------------------------------
    void GraphicsSystem::syncCameraProjection( bool bForceUpdate )
    {
        if( bForceUpdate && mVrData )
        {
            Ogre::Matrix4 eyeToHead[2];
            Ogre::Matrix4 projectionMatrix[2];
            Ogre::Matrix4 projectionMatrixRS[2];
            Ogre::Vector4 eyeFrustumExtents[2];

            for( size_t i=0u; i<2u; ++i )
            {
                if (mHMD)
                {
                    vr::EVREye eyeIdx = static_cast<vr::EVREye>( i );
                    eyeToHead[i] = convertSteamVRMatrixToMatrix(
                        mHMD->GetEyeToHeadTransform( eyeIdx ) );
                    projectionMatrix[i] =
                            convertSteamVRMatrixToMatrix(
                                mHMD->GetProjectionMatrix( eyeIdx,
                                mCamNear, mCamFar ) );
                    mHMD->GetProjectionRaw(
                        eyeIdx,
                        &eyeFrustumExtents[i].x, &eyeFrustumExtents[i].y,
                        &eyeFrustumExtents[i].z, &eyeFrustumExtents[i].w );
                }
//                 else if()
//                 {
//                      // TODO we are creating our own eyeToHead Matrix and projectionMatrix
//                 }
                else
                {
                    projectionMatrix[i] = mHmdConfig.projectionMatrix[i];
                    eyeToHead[i] = mHmdConfig.eyeToHead[i];
                    eyeFrustumExtents[i] = mHmdConfig.tan[i];
                }
                mRoot->getRenderSystem()->_convertOpenVrProjectionMatrix(
                    projectionMatrix[i], projectionMatrixRS[i] );

                LOG<< "eyeToHead"<< LOGEND;
                printMatrix4(eyeToHead[i]);
                LOG<< "projectionMatrix"<< LOGEND;
                printMatrix4(projectionMatrix[i]);
                LOG<< "projectionMatrixRS"<< LOGEND;
                printMatrix4(projectionMatrixRS[i]);
            }
//             mVrData->set( eyeToHead, projectionMatrixRS );
            mVrData->set( eyeToHead, projectionMatrixRS );

            Ogre::Vector4 cameraCullFrustumExtents;
            cameraCullFrustumExtents.x = std::min(
                eyeFrustumExtents[0].x, eyeFrustumExtents[1].x );
            cameraCullFrustumExtents.y = std::max(
                eyeFrustumExtents[0].y, eyeFrustumExtents[1].y );
            cameraCullFrustumExtents.z = std::max(
                eyeFrustumExtents[0].z, eyeFrustumExtents[1].z );
            cameraCullFrustumExtents.w = std::min(
                eyeFrustumExtents[0].w, eyeFrustumExtents[1].w );

            mVrCullCamera->setFrustumExtents(
                cameraCullFrustumExtents.x,
                cameraCullFrustumExtents.y,
                cameraCullFrustumExtents.w,
                cameraCullFrustumExtents.z,
                Ogre::FET_TAN_HALF_ANGLES );

            const float ipd = mVrData->mLeftToRight.x;
            Ogre::Vector3 cullCameraOffset = Ogre::Vector3::ZERO;
            cullCameraOffset.z = (ipd / 2.0f) /
                Ogre::Math::Abs( cameraCullFrustumExtents.x );

            const Ogre::Real offset = cullCameraOffset.length();
            mVrCullCamera->setNearClipDistance( mCamNear + offset );
            mVrCullCamera->setFarClipDistance( mCamFar + offset );
        }
    }

    Ogre::Vector4 GraphicsSystem::getVpOffset( Distortion dist, size_t eye)
    {
        size_t vr_width_half = mVrTexture->getWidth()/2;
        size_t vr_width = mVrTexture->getWidth();
        size_t vr_height = mVrTexture->getHeight();
        float f_x, f_y, c_x, c_y;
        StereoCameraConfig cameraConfig = mVideoLoader->getStereoCameraConfig();
        switch(dist)
        {
            case DIST_RAW:
                f_x = cameraConfig.cfg[eye].K[0];
                f_y = cameraConfig.cfg[eye].K[4];
                c_x = cameraConfig.cfg[eye].K[2];
                c_y = cameraConfig.cfg[eye].K[5];
                break;
            case DIST_UNDISTORT_RECTIFY:
                f_x = cameraConfig.cfg[eye].P[0];
                f_y = cameraConfig.cfg[eye].P[5];
                c_x = cameraConfig.cfg[eye].P[2];
                c_y = cameraConfig.cfg[eye].P[6];
                break;
            default:
                return Ogre::Vector4(0,0,0,0);
        }
        float img_width = cameraConfig.cfg[eye].width;
        float img_height = cameraConfig.cfg[eye].height;

        float c_vr_h = -mHmdConfig.tan[eye][0] /
            (-mHmdConfig.tan[eye][0] + mHmdConfig.tan[eye][1]);
        float c_vr_v = -mHmdConfig.tan[eye][2] /
            (-mHmdConfig.tan[eye][2] + mHmdConfig.tan[eye][3]);

        float img_size_resize_h = c_vr_h * img_width /
            (f_x *-mHmdConfig.tan[eye][0]);
        float img_size_resize_v = c_vr_v * img_height /
            (f_y *-mHmdConfig.tan[eye][2]);

        float img_middle_resize_h =
            img_size_resize_h * c_x / img_width;
        float img_middle_resize_v =
            img_size_resize_v * c_y / img_height;

        float align_f_h = (c_vr_h - img_middle_resize_h)/2
            + ( eye == LEFT ? 0 : 0.5 );
        float align_f_v = c_vr_v - img_middle_resize_v;
        float img_size_resize_h_entire = img_size_resize_h/2;
        return Ogre::Vector4(
            align_f_h, align_f_v, img_size_resize_h_entire, img_size_resize_v );
    }

    bool GraphicsSystem::calcAlign()
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();

        // for some reason we can only update every n frames.
        //which is limited by this function
        mUpdateFrames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();

        if (mHMD)
        {
            mHMD->GetProjectionRaw(
                vr::Eye_Left,
                &mHmdConfig.tan[LEFT][0], &mHmdConfig.tan[LEFT][1],
                &mHmdConfig.tan[LEFT][2], &mHmdConfig.tan[LEFT][3]);
            mHMD->GetProjectionRaw(
                vr::Eye_Right,
                &mHmdConfig.tan[RIGHT][0], &mHmdConfig.tan[RIGHT][1],
                &mHmdConfig.tan[RIGHT][2], &mHmdConfig.tan[RIGHT][3]);
        }

        Ogre::Matrix4 eyeToHead[2];
        Ogre::Matrix4 proj_matrix[2];
        size_t vr_width_half = mVrTexture->getWidth()/2;
        size_t vr_width = mVrTexture->getWidth();
        size_t vr_height = mVrTexture->getHeight();

        StereoCameraConfig cameraConfig = mVideoLoader->getStereoCameraConfig();
        for (size_t eye = 0; eye < mEyeNum; eye++)
        {
            float far_plane = 100.0;
            float near_plane = 0.001;
            float img_width = cameraConfig.cfg[eye].width;
            float img_height = cameraConfig.cfg[eye].height;
            float f_x = cameraConfig.cfg[eye].P[0];
            float f_y = cameraConfig.cfg[eye].P[5];
            float c_x = cameraConfig.cfg[eye].P[2];
            float c_y = cameraConfig.cfg[eye].P[6];
            float win_width = mHmdConfig.width;
            float win_height = mHmdConfig.height;
            float zoom_x = 1.0f;
            float zoom_y = zoom_x;

            // Preserve aspect ratio
//                 if (win_width != 0 && win_height != 0)
//                 {
//                     float img_aspect = (img_width / f_x) / (img_height / f_y);
//                     float win_aspect = win_width / win_height;
// 
//                     if (img_aspect > win_aspect)
//                     {
//                         zoom_y = zoom_y / img_aspect * win_aspect;
//                     }
//                     else
//                     {
//                         zoom_x = zoom_x / win_aspect * img_aspect;
//                     }
//                 }

            eyeToHead[eye] = Ogre::Matrix4::IDENTITY;
            proj_matrix[eye] = Ogre::Matrix4::ZERO;

            proj_matrix[eye][0][0] = 2.0 * f_x / img_width * zoom_x;
            proj_matrix[eye][1][1] = 2.0 * f_y / img_height * zoom_y;

            proj_matrix[eye][0][2] = 2.0 * (0.5 - c_x / img_width) * zoom_x;
            proj_matrix[eye][1][2] = 2.0 * (c_y / img_height - 0.5) * zoom_y;

            proj_matrix[eye][2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
            proj_matrix[eye][2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

            proj_matrix[eye][3][2] = -1;
            // we have also to set position and orientation
//                 mEyeCameras[eye]->setCustomProjectionMatrix( true, proj_matrix);
            //2.0f ~= cameraConfig.cfg[eye].P[0]
            // x   ~= cameraConfig.cfg[eye].P[3]
            eyeToHead[eye][0][3] = cameraConfig.cfg[eye].P[3] *2.0f / cameraConfig.cfg[eye].P[0];
//                 mEyeCameras[eye]->setPosition(position );
//                 Ogre::Vector3 focusPoint = Ogre::Vector3( 0.0f, 0.0f, -1.0f );
//                 mCameraNode[eye]->lookAt( focusPoint, Ogre::Node::TS_LOCAL );
            Ogre::Matrix4 proj_matrix_rs;
            mRoot->getRenderSystem()->_convertOpenVrProjectionMatrix(
                proj_matrix[eye], proj_matrix_rs );

            mEyeCameras[eye]->setCustomProjectionMatrix( true, proj_matrix[eye] );
//                 mEyeCameras[eye]->setAutoAspectRatio( true );
            mEyeCameras[eye]->setNearClipDistance( 0.0001 );
            mEyeCameras[eye]->setFarClipDistance( mCamFar );
            double tx = (cameraConfig.cfg[eye].P[3] / f_x);
            Ogre::Vector3 position = Ogre::Vector3::UNIT_X * tx;
            LOG << "Pos "<< cameraConfig.cfg[eye].eye_str << eye << ": " << position.x << " " << position.y << " " <<position.z <<LOGEND;
            mEyeCameras[eye]->setPosition( position );
            mEyeCameras[eye]->lookAt(0,0,1);
        }
        // now as we have camera config we use it.
        mVrData->set( eyeToHead, proj_matrix);
        return true;
    }

    Ogre::CompositorWorkspace* GraphicsSystem::setupCompositor(void)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        initOpenVR();
        calcAlign();
        //create Workspaces
        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {
            createTwoWorkspaces();
        }
        else if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mVrWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager, mVrTexture,
                mCamera, "InstancedStereoWorkspace", true, 0 );
        }

        mOvrCompositorListener =
            new OpenVRCompositorListener(
                mHMD, mVRCompositor, mVrTexture,
                mRoot, mVrWorkspaces,
                mCamerasNode, mCameraPoseState
            );
        setupImageData();

        syncCameraProjection( true );

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
    std::string GraphicsSystem::GetTrackedDeviceString(
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

    void GraphicsSystem::initCompositorVR(void)
    {
        mVRCompositor = vr::VRCompositor();
        if ( !mVRCompositor )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_RENDERINGAPI_ERROR,
                         "VR Compositor initialization failed. See log file for details",
                         "StereoRenderingGraphicsSystem::initCompositorVR" );
        }
    }

    void GraphicsSystem::initOpenVR(void)
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
            width = mHmdConfig.width;
            height = mHmdConfig.height;
//          maybe we say that render windown
//             width = mRenderWindow->getWidth();
//             height = mRenderWindow->getHeight();
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

    }

    void GraphicsSystem::createTwoWorkspaces()
    {
        Ogre::CompositorManager2 *compositorManager =
            mRoot->getCompositorManager2();
        const Ogre::IdString workspaceName( "TwoCamerasWorkspace" );
        Ogre::uint8 vpModifierMask, executionMask;
        Ogre::Vector4 vpOffsetScale;

        vpModifierMask  = 0x01;
        executionMask   = 0x01;
        //set offset so that we only render to the portion of the screen where there is the image
        vpOffsetScale =  getVpOffset( mVideoLoader->getDistortion(), LEFT );
        mVrWorkspaces[LEFT] = compositorManager->addWorkspace(
            mSceneManager,
            mVrTexture,
            mEyeCameras[LEFT], workspaceName,
            true, -1, (Ogre::UavBufferPackedVec*)0,
            (Ogre::ResourceLayoutMap*)0,
            (Ogre::ResourceAccessMap*)0,
            vpOffsetScale,
            vpModifierMask,
            executionMask );

        vpModifierMask  = 0x02;
        executionMask   = 0x02;
        vpOffsetScale =  getVpOffset( mVideoLoader->getDistortion(), RIGHT );
        mVrWorkspaces[RIGHT] = compositorManager->addWorkspace(
            mSceneManager,
            mVrTexture,
            mEyeCameras[RIGHT], workspaceName,
            true, -1, (Ogre::UavBufferPackedVec*)0,
            (Ogre::ResourceLayoutMap*)0,
            (Ogre::ResourceAccessMap*)0,
            vpOffsetScale,
            vpModifierMask,
            executionMask);
    }

    void GraphicsSystem::setupImageData()
    {
        if (!mVrWorkspaces[LEFT])
            return;
        Ogre::TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();

        if ( mVideoTarget == VRT_TO_2D_RECTANGLE )
        {
            if ( mIsStereo )
            {
                //TODO: guard if we don't find it.
                LOG << "setup Video Texture Left" << LOGEND;
                mVideoTexture[LEFT] = mVrWorkspaces[LEFT]->findNode("TwoCamerasNode")->getLocalTextures()[0];
                mVideoTexture[RIGHT] = mVrWorkspaces[RIGHT]->findNode("TwoCamerasNode")->getLocalTextures()[1];
            }
            else
            {
                mVideoTexture[LEFT] = textureManager->createTexture(
                    "VideoTextureMono",
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::AutomaticBatching |
                    Ogre::TextureFlags::ManualTexture |
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D );
            }
        }
        for ( size_t eye = 0; eye < mEyeNum; eye++)
        {
            const Ogre::uint32 rowAlignment = 4u;
            Ogre::PixelFormatGpu format = mVideoTexture[eye]->getPixelFormat();
            size_t imageDataSize =
            Ogre::PixelFormatGpuUtils::getSizeBytes(
                mVideoTexture[eye]->getWidth(),
                mVideoTexture[eye]->getHeight(),
                mVideoTexture[eye]->getDepth(),
                mVideoTexture[eye]->getNumSlices(),
                format,
                rowAlignment );
            if(!Ogre::PixelFormatGpuUtils::isCompressed(format) &&
                !mVideoLoader->updateDestinationSize(
                    mVideoTexture[eye]->getWidth(),
                    mVideoTexture[eye]->getHeight(),
                    Ogre::PixelFormatGpuUtils::getBytesPerPixel( format ),
                    imageDataSize ) )
            {
                setQuit();
            }
            mStagingTexture[eye] =
            textureManager->getStagingTexture(
                mVideoTexture[eye]->getWidth(),
                mVideoTexture[eye]->getHeight(),
                mVideoTexture[eye]->getDepth(),
                mVideoTexture[eye]->getNumSlices(),
                mVideoTexture[eye]->getPixelFormat() );
        }
    }

    void GraphicsSystem::setDistortion(Distortion dist)
    {
        if( mVrWorkspaces[LEFT] && mVrWorkspaces[RIGHT] )
        {
            for(size_t eye = 0; eye < mEyeNum; eye++)
            {
                mVrWorkspaces[eye]->setViewportModifier(getVpOffset(dist, eye));
            }
        }
        mVideoLoader->setDistortion(dist);
    }

    GraphicsSystem::GraphicsSystem(
            Demo::GameState* gameState,
            WorkspaceType wsType,
            Ogre::VrData *vrData,
            HmdConfig hmdConfig,
            VideoLoader *videoLoader,
            PoseState *poseState,
            int screen,
            bool isStereo,
            bool showOgreConfigDialog,
            bool showVideo,
            VideoRenderTarget renderVideoTarget,
            Ogre::Real camNear, Ogre::Real camFar ) :
        Demo::GraphicsSystem( gameState, showOgreConfigDialog,
                        RESOURCE_FOLDER, PLUGIN_FOLDER ),
        mWorkSpaceType( wsType ),
        mCamerasNode( nullptr ),
        mCameraNode{ nullptr, nullptr },
        mCameraPoseState( poseState ),
        mEyeCameras{ nullptr, nullptr },
        mCamNear( camNear ),
        mCamFar( camFar ),
        mVrWorkspaces{ nullptr, nullptr },
        mMirrorWorkspace( nullptr ),
        mVrCullCamera( nullptr ),
        mVrTexture( nullptr ),
        mVideoTexture{ nullptr, nullptr },
        mVrData( vrData ),
        mHmdConfig( hmdConfig ),
        mOvrCompositorListener( nullptr ),
        mHMD( nullptr ),
        mVRCompositor( nullptr ),
        mStrDriver( "" ),
        mStrDisplay( "" ),
        mDeviceModelNumber( "" ),
        mVideoLoader( videoLoader ),
        mVideoTarget( renderVideoTarget ),
        mStagingTexture{ nullptr, nullptr },
        mScreen( screen ),
        mIsStereo( isStereo ),
        mEyeNum( isStereo ? 2 : 1 ),
        mShowVideo(showVideo),
        mLastFrameUpdate(0),
        mUpdateFrames(2)
    {
        LOG << "RESOURCE_FOLDER:" << RESOURCE_FOLDER << LOGEND;
        LOG << "PLUGIN_FOLDER:" << PLUGIN_FOLDER << LOGEND;
        memset( mTrackedDevicePose, 0, sizeof (mTrackedDevicePose) );
    }

    //-----------------------------------------------------------------------------------
    // Just little bit modified code from GraphicsSystem
    void GraphicsSystem::initialize( const Ogre::String &windowTitle )
    {
    #if OGRE_USE_SDL2
        //if( SDL_Init( SDL_INIT_EVERYTHING ) != 0 )
        if( SDL_Init( SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
                      SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS ) != 0 )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR, "Cannot initialize SDL2!",
                         "GraphicsSystem::initialize" );
        }
    #endif

        Ogre::String pluginsPath;
        // only use plugins.cfg if not static
    #ifndef OGRE_STATIC_LIB
    #if OGRE_DEBUG_MODE
        pluginsPath = mPluginsFolder + "plugins_d.cfg";
    #else
        pluginsPath = mPluginsFolder + "plugins.cfg";
    #endif
    #endif

        mRoot = OGRE_NEW Ogre::Root( pluginsPath,
                                     mWriteAccessFolder + "ogre.cfg",
                                     mWriteAccessFolder + "Ogre.log" );

        mStaticPluginLoader.install( mRoot );

        // enable sRGB Gamma Conversion mode by default for all renderers,
        // but still allow to override it via config dialog
        Ogre::RenderSystemList::const_iterator itor = mRoot->getAvailableRenderers().begin();
        Ogre::RenderSystemList::const_iterator endt = mRoot->getAvailableRenderers().end();

        while( itor != endt )
        {
            Ogre::RenderSystem *rs = *itor;
            rs->setConfigOption( "sRGB Gamma Conversion", "Yes" );
            ++itor;
        }

        if( mAlwaysAskForConfig || !mRoot->restoreConfig() )
        {
            if( !mRoot->showConfigDialog() )
            {
                setQuit();
                return;
            }
        }

        mRoot->initialise( false, windowTitle );

        Ogre::ConfigOptionMap& cfgOpts = mRoot->getRenderSystem()->getConfigOptions();

        int width   = 2160;
        int height  = 1200;

        Ogre::ConfigOptionMap::iterator opt = cfgOpts.find( "Video Mode" );
        if( opt != cfgOpts.end() )
        {
            //Ignore leading space
            const Ogre::String::size_type start = opt->second.currentValue.find_first_of("012356789");
            //Get the width and height
            Ogre::String::size_type widthEnd = opt->second.currentValue.find(' ', start);
            // we know that the height starts 3 characters after the width and goes until the next space
            Ogre::String::size_type heightEnd = opt->second.currentValue.find(' ', widthEnd+3);
            // Now we can parse out the values
            width   = Ogre::StringConverter::parseInt(
                opt->second.currentValue.substr( 0, widthEnd ) );
            height  = Ogre::StringConverter::parseInt(
                opt->second.currentValue.substr( widthEnd+3, heightEnd ) );
        }

        Ogre::NameValuePairList params;
        bool fullscreen = Ogre::StringConverter::parseBool(
            cfgOpts["Full Screen"].currentValue );
    #if OGRE_USE_SDL2
        if (mScreen >= SDL_GetNumVideoDisplays())
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR,
                         "Requested Screen does not exist",
                         "GraphicsSystem::initialize" );
        }
        int posX = SDL_WINDOWPOS_CENTERED_DISPLAY(mScreen);
        int posY = SDL_WINDOWPOS_CENTERED_DISPLAY(mScreen);

        if(fullscreen)
        {
            posX = SDL_WINDOWPOS_UNDEFINED_DISPLAY(mScreen);
            posY = SDL_WINDOWPOS_UNDEFINED_DISPLAY(mScreen);
            SDL_DisplayMode dm;

            if (SDL_GetDesktopDisplayMode(mScreen, &dm) != 0)
            {
                SDL_Log("SDL_GetDesktopDisplayMode failed: %s", SDL_GetError());
            }

            width = dm.w;
            height = dm.h;
        }

        Uint32 flags =
            SDL_WINDOW_SHOWN  |
            SDL_WINDOW_OPENGL |
//             SDL_WINDOW_FULLSCREEN_DESKTOP |
            SDL_WINDOW_BORDERLESS;

        mSdlWindow = SDL_CreateWindow(
                    windowTitle.c_str(),    // window title
                    posX,               // initial x position
                    posY,               // initial y position
                    width,              // width, in pixels
                    height,             // height, in pixels
                    flags);

        //Get the native whnd
        SDL_SysWMinfo wmInfo;
        SDL_VERSION( &wmInfo.version );

        if( SDL_GetWindowWMInfo( mSdlWindow, &wmInfo ) == SDL_FALSE )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR,
                         "Couldn't get WM Info! (SDL2)",
                         "GraphicsSystem::initialize" );
        }

        Ogre::String winHandle;
        switch( wmInfo.subsystem )
        {
        #if defined(SDL_VIDEO_DRIVER_WINDOWS)
        case SDL_SYSWM_WINDOWS:
            // Windows code
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.win.window );
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_WINRT)
        case SDL_SYSWM_WINRT:
            // Windows code
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.winrt.window );
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_COCOA)
        case SDL_SYSWM_COCOA:
            winHandle  = Ogre::StringConverter::toString(WindowContentViewHandle(wmInfo));
            break;
        #endif
        #if defined(SDL_VIDEO_DRIVER_X11)
        case SDL_SYSWM_X11:
            winHandle = Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.x11.window );
            break;
        #endif
        default:
            OGRE_EXCEPT( Ogre::Exception::ERR_NOT_IMPLEMENTED,
                         "Unexpected WM! (SDL2)",
                         "GraphicsSystem::initialize" );
            break;
        }

        #if OGRE_PLATFORM == OGRE_PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WINRT
            params.insert( std::make_pair("externalWindowHandle",  winHandle) );
        #else
            params.insert( std::make_pair("parentWindowHandle",  winHandle) );
        #endif
    #endif

        params.insert( std::make_pair("title", windowTitle) );
        params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
        params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
        params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
        params.insert( std::make_pair("reverse_depth", "Yes" ) );

        initMiscParamsListener( params );

        mRenderWindow = Ogre::Root::getSingleton().createRenderWindow( windowTitle, width, height,
                                                                       fullscreen, &params );

        mOverlaySystem = OGRE_NEW Ogre::v1::OverlaySystem();

        setupResources();
        loadResources();
        chooseSceneManager();
        createCamera();
        mWorkspace = setupCompositor();

    #if OGRE_USE_SDL2
        mInputHandler = new Demo::SdlInputHandler( mSdlWindow, mCurrentGameState,
                                             mCurrentGameState, mCurrentGameState );
    #endif

        BaseSystem::initialize();

#if OGRE_PROFILING
        Ogre::Profiler::getSingleton().setEnabled( true );
    #if OGRE_PROFILING == OGRE_PROFILING_INTERNAL
        Ogre::Profiler::getSingleton().endProfile( "" );
    #endif
    #if OGRE_PROFILING == OGRE_PROFILING_INTERNAL_OFFLINE
        Ogre::Profiler::getSingleton().getOfflineProfiler().setDumpPathsOnShutdown(
                    mWriteAccessFolder + "ProfilePerFrame",
                    mWriteAccessFolder + "ProfileAccum" );
    #endif
#endif
    }

    void GraphicsSystem::deinitialize(void)
    {

        delete mOvrCompositorListener;
        mOvrCompositorListener = 0;

        Ogre::TextureGpuManager *textureManager =
        mRoot->getRenderSystem()->getTextureGpuManager();

        if( mVrTexture )
        {
            textureManager->destroyTexture( mVrTexture );
            mVrTexture = 0;
        }

        for( size_t eye = 0; eye < 2; eye++ )
        {
            if( mStagingTexture[eye] )
            {
                //Tell the TextureGpuManager we're done with this StagingTexture. Otherwise it will leak.
                textureManager->removeStagingTexture( mStagingTexture[eye] );
                mStagingTexture[eye] = 0;
            }

            //Don't need to be destroyed compositorManager is doing it
            mVideoTexture[eye] = 0;

            if( mEyeCameras[eye] )
            {
                mSceneManager->destroyCamera( mEyeCameras[eye] );
                mEyeCameras[eye] = nullptr;
            }
        }

        if( mVrCullCamera )
        {
            mSceneManager->destroyCamera( mVrCullCamera );
            mVrCullCamera = 0;
        }

        if( mHMD )
        {
            vr::VR_Shutdown();
            mHMD = NULL;
        }

        Demo::GraphicsSystem::deinitialize();
    }

    void GraphicsSystem::beginFrameParallel(void)
    {
        BaseSystem::beginFrameParallel();
        if ( mOvrCompositorListener->getFrameCnt() >=
                mLastFrameUpdate + mUpdateFrames &&
            mVideoTexture && mStagingTexture[LEFT])
        {
            mLastFrameUpdate = mOvrCompositorListener->getFrameCnt();
//             LOG << "update" << LOGEND;
            StereoImageData imgData = mVideoLoader->getCurStereoImageData();
            for( size_t eye = 0; eye < mEyeNum; eye++ )
            {
                const size_t bytesPerRow =
                    mVideoTexture[eye]->_getSysRamCopyBytesPerRow( 0 );

                mStagingTexture[eye]->startMapRegion();
                Ogre::TextureBox texBox = mStagingTexture[eye]->mapRegion(
                    mVideoTexture[eye]->getWidth(),
                    mVideoTexture[eye]->getHeight(),
                    mVideoTexture[eye]->getDepth(),
                    mVideoTexture[eye]->getNumSlices(),
                    mVideoTexture[eye]->getPixelFormat() );
                texBox.copyFrom( imgData.img[eye].data,
                                mVideoTexture[eye]->getWidth(),
                                mVideoTexture[eye]->getHeight(),
                                bytesPerRow );
                mStagingTexture[eye]->stopMapRegion();
                mStagingTexture[eye]->upload(
                    texBox, mVideoTexture[eye], 0, 0, 0, false );

                mVideoTexture[eye]->notifyDataIsReady();
            }
        }
        else
        {
//             LOG << "wait" << LOGEND;
        }
        if ( mCameraPoseState && mCameraPoseState->validPose() )
        {
            Ogre::Quaternion prev_orientation =
                mCamerasNode->getOrientation();
            Ogre::Vector3 prev_position = mCamerasNode->getPosition();
            Ogre::Quaternion orientation =
                mCameraPoseState->getOrientation();
            orientation = orientation * Ogre::Quaternion(
                Ogre::Degree(180), Ogre::Vector3::UNIT_Z);
            Ogre::Vector3 position = mCameraPoseState->getPosition();
            if ( !prev_orientation.orientationEquals(orientation)
                || prev_position != position )
            {
//                 LOG << "update orientation and position" << mOvrCompositorListener->getFrameCnt();
//                 LOG << "Pos: " << position.x << " " << position.y << " " <<position.z;
//                 LOG << "Orientation: " << orientation.w << " "<< orientation.x << " " << orientation.y << " " <<orientation.z<<LOGEND;
                mCamerasNode->setPosition( position );
                mCamerasNode->setOrientation( orientation );
            }
        }
    }
}
