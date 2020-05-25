#include "StereoRenderingGraphicsSystem.h"


#include "OgrePlatform.h"
#include "StereoRendering.h"
#include "OpenVRCompositorListener.h"

#include "GameState.h"

#include "OgreTextureGpuManager.h"
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
// #include "Compositor/OgreCompositorManager2.h"

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
        mVrCullCamera = mSceneManager->createCamera( "VrCullCamera" );
        mVrCullCamera->detachFromParent();
        mCamerasNode->attachObject( mVrCullCamera );

        //setup mVrData und mVrCullCamera using the mHmdConfig
        syncCameraProjection( true );

        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {
            mEyeCameras[LEFT] = mSceneManager->createCamera( "Left Eye" );
            mEyeCameras[RIGHT] = mSceneManager->createCamera( "Right Eye" );

//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;

            for( int leftOrRight = 0; leftOrRight < 2; ++leftOrRight )
            {
//                 const Ogre::Vector3 camPos( eyeDistance * (leftOrRight * 2 - 1), 0, 0 );

                Ogre::Matrix4 eyeToHead =
                    mVrData->mHeadToEye[leftOrRight].inverse();
                Ogre::Vector4 camPos = eyeToHead *
                    Ogre::Vector4( 0, 0, 0, 1.0 );
                // Look back along -Z
                Ogre::Vector4 camDir = eyeToHead *
                    Ogre::Vector4( 0, 0, -1.0, 0 );

                mEyeCameras[leftOrRight]->setPosition( camPos.xyz() );

//                 Ogre::Vector3 lookAt( eyeFocusDistance * (leftOrRight * 2 - 1), 0, -1 );
                //Ogre::Vector3 lookAt( 0, 0, 0 );

                mEyeCameras[leftOrRight]->setDirection( camDir.xyz() );
                mEyeCameras[leftOrRight]->setNearClipDistance( mCamNear );
                mEyeCameras[leftOrRight]->setFarClipDistance( mCamFar );
                mEyeCameras[leftOrRight]->setCustomProjectionMatrix( true, mVrData->mProjectionMatrix[leftOrRight] );
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

    //-------------------------------------------------------------------------
    void StereoGraphicsSystem::syncCameraProjection( bool bForceUpdate )
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

    bool StereoGraphicsSystem::calcAlign(CameraConfig &cameraConfig)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();

        // for some reason we can only update every n frames.
        //which is limited by this function
        mUpdateFrames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();

        for ( size_t eye = 0; eye < mEyeNum; eye++)
        {
            mCameraWidth[eye] = cameraConfig.width[eye];
            mCameraHeight[eye] = cameraConfig.height[eye];

            if ( mVideoTarget == TO_SQUARE )
            {
                mVideoTexture[eye]->setResolution(
                    mCameraWidth[eye], mCameraHeight[eye] );
                mVideoTexture[eye]->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
            }

            const Ogre::uint32 rowAlignment = 4u;
            mImageDataSize[eye] =
                Ogre::PixelFormatGpuUtils::getSizeBytes(
                    mVideoTexture[eye]->getWidth(),
                    mVideoTexture[eye]->getHeight(),
                    mVideoTexture[eye]->getDepth(),
                    mVideoTexture[eye]->getNumSlices(),
                    mVideoTexture[eye]->getPixelFormat(),
                    rowAlignment );

            mImageData[eye] = reinterpret_cast<Ogre::uint8*>(
                OGRE_MALLOC_SIMD( mImageDataSize[eye],
                Ogre::MEMCATEGORY_RESOURCE ) );
            memset(mImageData[eye], 0, mImageDataSize[eye]);

            if ( mVideoTarget == TO_SQUARE )
            {
                mVideoTexture[eye]->scheduleTransitionTo( Ogre::GpuResidency::Resident );
            }

            mStagingTexture[eye] =
                textureManager->getStagingTexture(
                    mVideoTexture[eye]->getWidth(),
                    mVideoTexture[eye]->getHeight(),
                    mVideoTexture[eye]->getDepth(),
                    mVideoTexture[eye]->getNumSlices(),
                    mVideoTexture[eye]->getPixelFormat() );
        }

        //now we have to know
        if (!mImageRenderConfig)
        {
            mImageRenderConfig = new ImageRenderConfig();
        }

        size_t vr_width_half = mVrTexture->getWidth()/2;
        size_t vr_height = mVrTexture->getHeight();

//              left left
//                 = -1.39377;
//              left right
//                 = 1.23437;
//              left top
//                  = -1.46653 ;
//              left bottom
//                 = 1.45802 ;
//              right left
//                 = -1.24354 ;
//              right right
//                 = 1.39482;
//              right top
//                 = -1.47209;
//              right bottom
//                 = 1.45965;
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

        for (size_t eye = 0; eye < mEyeNum; eye++)
        {
            float c_vr_h = -mHmdConfig.tan[eye][0] * vr_width_half /
                (-mHmdConfig.tan[eye][0] + mHmdConfig.tan[eye][1]);
            float c_vr_v = -mHmdConfig.tan[eye][2] * vr_height /
                (-mHmdConfig.tan[eye][2] + mHmdConfig.tan[eye][3]);

            float img_size_resize_h = c_vr_h * cameraConfig.width[eye] /
                (cameraConfig.f_x[eye] *-mHmdConfig.tan[eye][0]);
            float img_size_resize_v = c_vr_v * cameraConfig.height[eye] /
                (cameraConfig.f_y[eye] *-mHmdConfig.tan[eye][2]);

            float img_middle_resize_h =
                img_size_resize_h * cameraConfig.c_x[eye] / cameraConfig.width[eye];
            float img_middle_resize_v =
                img_size_resize_v * cameraConfig.c_y[eye] / cameraConfig.height[eye];

            float align_f_h = c_vr_h - img_middle_resize_h;
            float align_f_v = c_vr_v - img_middle_resize_v;

            if (align_f_h <= 0 || align_f_v <= 0 )
                return false;
            mImageRenderConfig->leftAlign[eye] = 
                (eye == RIGHT ? vr_width_half : 0) +
                static_cast<size_t>(std::round(align_f_h));
            mImageRenderConfig->topAlign[eye] =
                static_cast<size_t>(std::round(align_f_v));
            OGRE_ASSERT_MSG(img_size_resize_v > 0 && img_size_resize_h > 0, "img_height/width_resize smaller than/ equal zero");
            mImageRenderConfig->size[eye] = {
                static_cast<int>(std::round(img_size_resize_h)),
                static_cast<int>(std::round(img_size_resize_v)) };

            //for mDrawHelpers
            mCVr[eye][0] = static_cast<size_t>(std::round(c_vr_h));
            mCVr[eye][1] = static_cast<size_t>(std::round(c_vr_v));
            mImgMiddleResize[eye][0] = static_cast<size_t>(
                std::round(img_middle_resize_h));
            mImgMiddleResize[eye][1] = static_cast<size_t>(
                std::round(img_middle_resize_v));
        }
        return true;
    }

    Ogre::CompositorWorkspace* StereoGraphicsSystem::setupCompositor(void)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        initOpenVR();

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
            vpOffsetScale   = Ogre::Vector4( 0.5f, 0.0f, 0.5f, 1.0f );
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
        else if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mVrWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager, mVrTexture,
                mCamera, "InstancedStereoWorkspace", true, 0 );
        }

        mOvrCompositorListener =
            new Demo::OpenVRCompositorListener(
                mHMD, mVRCompositor, mVrTexture,
                mRoot, mVrWorkspaces,
                mCamerasNode
            );
    }

    void StereoGraphicsSystem::setupImageData()
    {
        Ogre::TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();

        if ( mVideoTarget == TO_BACKGROUND )
        {
            mVideoTexture[LEFT] = mVrTexture;
        }
        else if ( mVideoTarget == TO_SQUARE )
        {
            if ( mIsStereo )
            {
                mVideoTexture[LEFT] = textureManager->createTexture(
                    "VideoTextureLeft",
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::AutomaticBatching |
                    Ogre::TextureFlags::ManualTexture |
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D );
                mVideoTexture[RIGHT] = textureManager->createTexture(
                    "VideoTextureRight",
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::AutomaticBatching |
                    Ogre::TextureFlags::ManualTexture |
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D );
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
    }

    bool StereoGraphicsSystem::fillTexture(void)
    {
        const size_t bytesPerPixel = 4u;
        const size_t bytesPerRow =
            mVrTexture->_getSysRamCopyBytesPerRow( 0 );

//         LOG << mVideoTexture->getWidth() << LOGEND;
//         LOG << mVideoTexture->getHeight() << LOGEND;
//         LOG << width_resize << LOGEND;
//         LOG << "height_resize1 " << height_resize << LOGEND;
//         LOG << height_resize << LOGEND;

//         LOG << "width_resize " << width_resize << LOGEND;
//         LOG << "height_resize " << height_resize << LOGEND;
//         LOG << "ldst.cols " << ldst.cols << LOGEND;
//         LOG << "ldst.rows " << ldst.rows << LOGEND;

        if ( mImageResize[LEFT].empty() || mImageResize[RIGHT].empty() ||
            !mImageRenderConfig)
        {
            return false;
        }
        size_t align_left;
        size_t align_top;
        cv::Mat* dst;
        for(size_t eye = 0; eye < 2u; eye++) {
            align_left = mImageRenderConfig->leftAlign[eye];
            align_top = mImageRenderConfig->topAlign[eye];
            dst = &mImageResize[eye];
            size_t row_cnt = align_top * bytesPerRow;
            for (int y = 0; y < mImageRenderConfig->size[eye].height; y++) {
                size_t cnt = row_cnt + (align_left * bytesPerPixel);
                uint8_t* img_row_ptr = dst->ptr<uint8_t>(y);
                for (int x = 0; x < mImageRenderConfig->size[eye].width; x++) {
                    mImageData[0][cnt++] = *(img_row_ptr+2);
                    mImageData[0][cnt++] = *(img_row_ptr+1);
                    mImageData[0][cnt++] = *img_row_ptr;
                    img_row_ptr += 3;
                    mImageData[0][cnt++] = 0;
                }
                row_cnt += bytesPerRow;
            }
        }

        if (mDrawHelpers)
        {
            //redline for eye pupilar middle
            for (size_t i = 0; i < mVrTexture->getHeight(); i++)
            {
                mImageData[0][(bytesPerRow*i) + (mCVr[LEFT][0] * bytesPerPixel)] = 255;
                mImageData[0][(bytesPerRow*i) + (bytesPerRow/2) + (mCVr[RIGHT][0] * bytesPerPixel)+4] = 255;
            }

            //green line for middle
            for (size_t i = 0; i < mVrTexture->getHeight()*4; i++)
            {
                mImageData[0][(bytesPerRow/4*i)+1] = 255;
            }
        }
        return true;
    }

    StereoGraphicsSystem::StereoGraphicsSystem(
            GameState* gameState,
            WorkspaceType wsType,
            Ogre::VrData *vrData,
            HmdConfig hmdConfig,
            int screen,
            bool isStereo,
            bool showOgreConfigDialog,
            bool showVideo,
            Demo::VideoRenderTarget renderVideoTarget,
            Ogre::Real camNear, Ogre::Real camFar ) :
        GraphicsSystem( gameState, showOgreConfigDialog,
                        RESOURCE_FOLDER, PLUGIN_FOLDER ),
        mWorkSpaceType( wsType ),
        mCamerasNode( nullptr ),
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
        mVideoSource( nullptr ),
        mVideoTarget( renderVideoTarget ),
        mCameraWidth{ 0, 0 },
        mCameraHeight{ 0, 0 },
        mImageRenderConfig( nullptr ),
        mMtxImageResize(),
        mImageData{ nullptr, nullptr },
        mImageDataSize{ 0, 0 },
        mStagingTexture{ nullptr, nullptr },
        mDrawHelpers(true),
        mCVr{{ 0, 0 }, { 0, 0 }},
        mImgMiddleResize{{ 0, 0 }, { 0, 0 }},
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
    void StereoGraphicsSystem::initialize( const Ogre::String &windowTitle )
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
                mQuit = true;
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
        mInputHandler = new SdlInputHandler( mSdlWindow, mCurrentGameState,
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

    void StereoGraphicsSystem::deinitialize(void)
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

            if( mVideoTexture[eye] )
            {
                textureManager->destroyTexture( mVideoTexture[eye] );
                mVideoTexture[eye] = 0;
            }

            if( mImageData[eye] )
            {
                //Do not free the pointer if texture's paging strategy is GpuPageOutStrategy::AlwaysKeepSystemRamCopy
                OGRE_FREE_SIMD( mImageData[eye], Ogre::MEMCATEGORY_RESOURCE );
                mImageData[eye] = 0;
            }
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

        GraphicsSystem::deinitialize();
    }

    void StereoGraphicsSystem::setImgPtr(
        const cv::Mat *left, const cv::Mat *right)
    {
        const cv::Mat *img[2] = { left, right };
        if( !mShowVideo )
            return;
        //we have to wait some frames after we can send new texture
        // maybe we have to look this also applies to Hlms
        if ( !img[RIGHT] )
            img[RIGHT] = img[LEFT];
        if ( mVideoTarget == TO_BACKGROUND  && mImageRenderConfig )
        {
            //why don't we just fire images to the lens as we have them
            resize(*img[LEFT], mImageResize[LEFT], mImageRenderConfig->size[LEFT]);
            resize(*img[RIGHT], mImageResize[RIGHT], mImageRenderConfig->size[RIGHT]);

            if (mDrawHelpers)
            {
                circle( mImageResize[LEFT],
                    cv::Point(mImgMiddleResize[LEFT][0],mImgMiddleResize[LEFT][1]),
                    5, cv::Scalar( 0, 0, 255 ), -1);
                circle( mImageResize[RIGHT],
                    cv::Point(mImgMiddleResize[RIGHT][0],mImgMiddleResize[RIGHT][1]),
                    5, cv::Scalar( 0, 0, 255 ), -1);
            }
            mMtxImageResize.lock();
            fillTexture();
            mMtxImageResize.unlock();
        }
        else if ( mVideoTarget == TO_SQUARE )
        {
//             mOvrCompositorListener->getFrameCnt() % 256;

            const cv::Mat *img_ptr[2];
            if ( static_cast<size_t>( img[LEFT]->cols ) == mCameraWidth[LEFT] &&
                static_cast<size_t>(  img[LEFT]->rows ) == mCameraHeight[LEFT] &&
                static_cast<size_t>( img[RIGHT]->cols ) == mCameraWidth[RIGHT] &&
                static_cast<size_t>( img[RIGHT]->rows ) == mCameraHeight[RIGHT] )
            {
                img_ptr[LEFT] = img[LEFT];
                img_ptr[RIGHT] = img[RIGHT];
            }
            else
            {
                //TODO:probably not the most efficient methode
                for (size_t eye = 0; eye < mEyeNum; eye++ )
                {
                    resize(*(img[eye]), mImageResize[eye], cv::Size(mCameraWidth[eye], mCameraHeight[eye]));
                    img_ptr[eye] = &mImageResize[eye];
                }
            }

            mMtxImageResize.lock();
            for( size_t eye = 0; eye < mEyeNum; eye++ )
            {
                const size_t bytesPerRow =
                    mVideoTexture[eye]->_getSysRamCopyBytesPerRow( 0 );
                size_t row_cnt = 0;
                for (size_t y = 0; y < mCameraHeight[eye]; y++) {
                    size_t cnt = row_cnt;
                    const uint8_t* img_row_ptr = img_ptr[eye]->ptr<const uint8_t>(y);
                    for (size_t x = 0; x < mCameraWidth[eye]; x++) {
                        mImageData[eye][cnt++] = *(img_row_ptr+2);
                        mImageData[eye][cnt++] = *(img_row_ptr+1);
                        mImageData[eye][cnt++] = *img_row_ptr;
                        img_row_ptr += 3;
                        mImageData[eye][cnt++] = 0;
                    }
                    row_cnt += bytesPerRow;
                }
            }
            mMtxImageResize.unlock();
        }
    }

    //-------------------------------------------------------------------------
    bool StereoGraphicsSystem::clearTexture(void)
    {
        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            const size_t bytesPerRow =
                mVideoTexture[eye]->_getSysRamCopyBytesPerRow( 0 );

            memset(mImageData[eye], 0, mImageDataSize[eye]);
            mStagingTexture[eye]->startMapRegion();
            Ogre::TextureBox texBox = mStagingTexture[eye]->mapRegion(
                mVideoTexture[eye]->getWidth(),
                mVideoTexture[eye]->getHeight(),
                mVideoTexture[eye]->getDepth(),
                mVideoTexture[eye]->getNumSlices(),
                mVideoTexture[eye]->getPixelFormat() );
            texBox.copyFrom( mImageData,
                            mVideoTexture[eye]->getWidth(),
                            mVideoTexture[eye]->getHeight(),
                            bytesPerRow );
            mStagingTexture[eye]->stopMapRegion();
            mStagingTexture[eye]->upload( texBox, mVideoTexture[eye], 0, 0, 0, false );

            mVideoTexture[eye]->notifyDataIsReady();
        }
        return true;
    }

    void StereoGraphicsSystem::beginFrameParallel(void)
    {
        BaseSystem::beginFrameParallel();
        if ( mOvrCompositorListener->getFrameCnt() >=
                mLastFrameUpdate + mUpdateFrames &&
            mVideoTexture )
        {
            mLastFrameUpdate = mOvrCompositorListener->getFrameCnt();
//             LOG << "update" << LOGEND;
            mMtxImageResize.lock();
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
                texBox.copyFrom( mImageData[eye],
                                mVideoTexture[eye]->getWidth(),
                                mVideoTexture[eye]->getHeight(),
                                bytesPerRow );
                mStagingTexture[eye]->stopMapRegion();
                mStagingTexture[eye]->upload(
                    texBox, mVideoTexture[eye], 0, 0, 0, false );

                mVideoTexture[eye]->notifyDataIsReady();
            }
            mMtxImageResize.unlock();
        }
        else
        {
//             LOG << "wait" << LOGEND;
        }

    }
}
