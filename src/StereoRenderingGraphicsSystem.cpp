#include "StereoRenderingGraphicsSystem.h"

#include "OgrePlatform.h"
#include "StereoRendering.h"
#include "OpenVRCompositorListener.h"

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
            mCamera->setVrData( &mVrData );
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
        const Ogre::Real camNear = mEyeCameras[0]->getNearClipDistance();
        const Ogre::Real camFar  = mEyeCameras[0]->getFarClipDistance();

        if( mCamNear != camNear || mCamFar != camFar || bForceUpdate )
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
                                camNear, camFar ) );
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

            mVrData.set( eyeToHead, projectionMatrixRS );
            mCamNear = camNear;
            mCamFar = camFar;

            LOG << "camNear: " << camNear << LOGEND;

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

            const float ipd = mVrData.mLeftToRight.x;
            Ogre::Vector3 cullCameraOffset = Ogre::Vector3::ZERO;
            cullCameraOffset.z = (ipd / 2.0f) /
                Ogre::Math::Abs( cameraCullFrustumExtents.x );

            const Ogre::Real offset = cullCameraOffset.length();
            mVrCullCamera->setNearClipDistance( camNear + offset );
            mVrCullCamera->setFarClipDistance( camFar + offset );
        }
    }

    bool StereoGraphicsSystem::calcAlign(CameraConfig &cameraConfig)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();

        // for some reason we can only update every n frames.
        //which is limited by this function
        mUpdateFrames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();

        mCameraWidth = cameraConfig.width[0];
        mCameraHeight = cameraConfig.height[0];

        if ( mVideoTarget == TO_SQUARE )
        {
            mVideoTexture->setResolution( mCameraWidth, mCameraHeight );
            mVideoTexture->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
        }

        const Ogre::uint32 rowAlignment = 4u;
        mImageDataSize =
            Ogre::PixelFormatGpuUtils::getSizeBytes(
                mVideoTexture->getWidth(),
                mVideoTexture->getHeight(),
                mVideoTexture->getDepth(),
                mVideoTexture->getNumSlices(),
                mVideoTexture->getPixelFormat(),
                rowAlignment );

        mImageData = reinterpret_cast<Ogre::uint8*>(
            OGRE_MALLOC_SIMD( mImageDataSize,
            Ogre::MEMCATEGORY_RESOURCE ) );
        memset(mImageData, 0, mImageDataSize);

        if ( mVideoTarget == TO_SQUARE )
        {
            mVideoTexture->scheduleTransitionTo( Ogre::GpuResidency::Resident );
        }

        mStagingTexture =
            textureManager->getStagingTexture(
                mVideoTexture->getWidth(),
                mVideoTexture->getHeight(),
                mVideoTexture->getDepth(),
                mVideoTexture->getNumSlices(),
                mVideoTexture->getPixelFormat() );

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

        for (size_t eye = 0; eye < 2; eye++)
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

        //TODO:probably we need two if we have two cameras
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
            mVideoTexture = mVrTexture;
        }
        else if ( mVideoTarget == TO_SQUARE )
        {
            mVideoTexture = textureManager->createTexture(
                "VideoTexture",
                Ogre::GpuPageOutStrategy::Discard,
                Ogre::TextureFlags::AutomaticBatching |
                Ogre::TextureFlags::ManualTexture |
                Ogre::TextureFlags::Reinterpretable,
                Ogre::TextureTypes::Type2D );
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
                    mImageData[cnt++] = *(img_row_ptr+2);
                    mImageData[cnt++] = *(img_row_ptr+1);
                    mImageData[cnt++] = *img_row_ptr;
                    img_row_ptr += 3;
                    mImageData[cnt++] = 0;
                }
                row_cnt += bytesPerRow;
            }
        }

        if (mDrawHelpers)
        {
            //redline for eye pupilar middle
            for (size_t i = 0; i < mVrTexture->getHeight(); i++)
            {
                mImageData[(bytesPerRow*i) + (mCVr[LEFT][0] * bytesPerPixel)] = 255;
                mImageData[(bytesPerRow*i) + (bytesPerRow/2) + (mCVr[RIGHT][0] * bytesPerPixel)+4] = 255;
            }

            //green line for middle
            for (size_t i = 0; i < mVrTexture->getHeight()*4; i++)
            {
                mImageData[(bytesPerRow/4*i)+1] = 255;
            }
        }
        return true;
    }

    StereoGraphicsSystem::StereoGraphicsSystem(
            GameState* gameState,
            WorkspaceType wsType,
            HmdConfig hmdConfig,
            bool showOgreConfigDialog,
            bool showVideo,
            Demo::VideoRenderTarget renderVideoTarget,
            Ogre::Real camNear, Ogre::Real camFar ) :
        GraphicsSystem( gameState, "../Data/" ),
        mWorkSpaceType( wsType ),
        mCamerasNode( nullptr ),
        mEyeCameras{ nullptr, nullptr },
        mCamNear( camNear ),
        mCamFar( camFar ),
        mVrWorkspaces{ nullptr, nullptr },
        mMirrorWorkspace( nullptr ),
        mVrCullCamera( nullptr ),
        mVrTexture( nullptr ),
        mVideoTexture( nullptr ),
        mHmdConfig( hmdConfig ),
        mOvrCompositorListener( nullptr ),
        mHMD( nullptr ),
        mVRCompositor( nullptr ),
        mStrDriver( "" ),
        mStrDisplay( "" ),
        mDeviceModelNumber( "" ),
        mVideoSource( nullptr ),
        mVideoTarget( renderVideoTarget ),
        mCameraWidth(0),
        mCameraHeight(0),
        mImageRenderConfig( nullptr ),
        mMtxImageResize(),
        mImageData( nullptr ),
        mImageDataSize( 0 ),
        mStagingTexture(nullptr),
        mDrawHelpers(true),
        mCVr{{ 0, 0 }, { 0, 0 }},
        mImgMiddleResize{{ 0, 0 }, { 0, 0 }},
        mShowVideo(showVideo),
        mLastFrameUpdate(0),
        mUpdateFrames(2)
    {
        memset( mTrackedDevicePose, 0, sizeof (mTrackedDevicePose) );
        memset( &mVrData, 0, sizeof( mVrData ) );
    }

    void StereoGraphicsSystem::deinitialize(void)
    {

        delete mOvrCompositorListener;
        mOvrCompositorListener = 0;

        if( mVrTexture )
        {
            Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
            textureManager->destroyTexture( mVrTexture );
            mVrTexture = 0;
        }

        if( mStagingTexture )
        {
            Ogre::TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();
            //Tell the TextureGpuManager we're done with this StagingTexture. Otherwise it will leak.
            textureManager->removeStagingTexture( mStagingTexture );
            mStagingTexture = 0;
        }

        if( mImageData )
        {
            //Do not free the pointer if texture's paging strategy is GpuPageOutStrategy::AlwaysKeepSystemRamCopy
            OGRE_FREE_SIMD( mImageData, Ogre::MEMCATEGORY_RESOURCE );
            mImageData = 0;
        }
        if( mEyeCameras[LEFT] )
        {
            mSceneManager->destroyCamera( mEyeCameras[LEFT] );
            mEyeCameras[LEFT] = nullptr;
        }
        if( mEyeCameras[RIGHT] )
        {
            mSceneManager->destroyCamera( mEyeCameras[RIGHT] );
            mEyeCameras[RIGHT] = nullptr;
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

    void StereoGraphicsSystem::setImgPtr(const cv::Mat *left, const cv::Mat *right)
    {
        if( !mShowVideo )
            return;
        //we have to wait some frames after we can send new texture
        // maybe we have to look this also applies to Hlms
        if ( !right )
            right = left;
        if ( mVideoTarget == TO_BACKGROUND  && mImageRenderConfig )
        {
            //why don't we just fire images to the lens as we have them
            resize(*left, mImageResize[LEFT], mImageRenderConfig->size[LEFT]);
            resize(*right, mImageResize[RIGHT], mImageRenderConfig->size[RIGHT]);

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
            const size_t bytesPerRow =
                mVideoTexture->_getSysRamCopyBytesPerRow( 0 );

            const cv::Mat *left_ptr;
            const cv::Mat *right_ptr;
            if ( left->cols == mCameraWidth &&
                left->rows == mCameraHeight &&
                right->cols == mCameraWidth &&
                right->rows == mCameraHeight )
            {
                left_ptr = left;
                right_ptr = right;
            }
            else
            {
                //TODO:probably not the most efficient methode
                resize(*left, mImageResize[LEFT], cv::Size(mCameraWidth, mCameraHeight));
                resize(*right, mImageResize[RIGHT], cv::Size(mCameraWidth, mCameraHeight));
                left_ptr = &mImageResize[LEFT];
                right_ptr = &mImageResize[RIGHT];
            }

            mMtxImageResize.lock();
            size_t row_cnt = 0;
            for (size_t y = 0; y < mCameraHeight; y++) {
                size_t cnt = row_cnt;
                const uint8_t* img_row_ptr = left_ptr->ptr<const uint8_t>(y);
                for (size_t x = 0; x < mCameraWidth; x++) {
                    mImageData[cnt++] = *(img_row_ptr+2);
                    mImageData[cnt++] = *(img_row_ptr+1);
                    mImageData[cnt++] = *img_row_ptr;
                    img_row_ptr += 3;
                    mImageData[cnt++] = 0;
                }
                row_cnt += bytesPerRow;
            }
            mMtxImageResize.unlock();
        }
    }

    //-------------------------------------------------------------------------
    bool StereoGraphicsSystem::clearTexture(void)
    {
        const size_t bytesPerRow =
            mVideoTexture->_getSysRamCopyBytesPerRow( 0 );

        memset(mImageData, 0, mImageDataSize);
        mStagingTexture->startMapRegion();
        Ogre::TextureBox texBox = mStagingTexture->mapRegion(
            mVrTexture->getWidth(),
            mVrTexture->getHeight(),
            mVrTexture->getDepth(),
            mVrTexture->getNumSlices(),
            mVrTexture->getPixelFormat() );
        texBox.copyFrom( mImageData,
                         mVrTexture->getWidth(),
                         mVrTexture->getHeight(),
                         bytesPerRow );
        mStagingTexture->stopMapRegion();
        mStagingTexture->upload( texBox, mVrTexture, 0, 0, 0, false );

        mVrTexture->notifyDataIsReady();
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
            LOG << "update" << LOGEND;
            mMtxImageResize.lock();
            const size_t bytesPerRow =
            mVideoTexture->_getSysRamCopyBytesPerRow( 0 );

            mStagingTexture->startMapRegion();
            Ogre::TextureBox texBox = mStagingTexture->mapRegion(
                mVideoTexture->getWidth(),
                mVideoTexture->getHeight(),
                mVideoTexture->getDepth(),
                mVideoTexture->getNumSlices(),
                mVideoTexture->getPixelFormat() );
            texBox.copyFrom( mImageData,
                            mVideoTexture->getWidth(),
                            mVideoTexture->getHeight(),
                            bytesPerRow );
            mStagingTexture->stopMapRegion();
            mStagingTexture->upload( texBox, mVideoTexture, 0, 0, 0, false );

            mVideoTexture->notifyDataIsReady();
            mMtxImageResize.unlock();
        }
        else
        {
            LOG << "wait" << LOGEND;
        }

    }
}
