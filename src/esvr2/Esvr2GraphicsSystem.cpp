#include "Esvr2GraphicsSystem.h"

#include "Esvr2.h"
#include "Esvr2OpenVRCompositorListener.h"

#include "Ogre.h"
#include "OgrePlatform.h"
#include "OgreTextureGpuManager.h"
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
#include "OgreResource.h"
#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorNode.h"
#include "Compositor/OgreCompositorWorkspace.h"

#include "OgrePixelFormatGpuUtils.h"
#include "OgreMemoryAllocatorConfig.h"
#include "OgreGpuResource.h"
#include "OgreStagingTexture.h"
#include "OgreHlmsDiskCache.h"
#include "Vao/OgreVaoManager.h"
#include <Overlay/OgreOverlayManager.h>

#include "Hlms/Unlit/OgreHlmsUnlit.h"
#include "Hlms/Pbs/OgreHlmsPbs.h"

#include "opencv2/opencv.hpp"
#include <sstream>
#include <cmath>
#include <mutex>

namespace esvr2
{
    GraphicsSystem::GraphicsSystem(
            Esvr2 *esvr2,
            std::shared_ptr<GameState> gameState):
            mEsvr2(esvr2),
            mGameState(gameState),
            mGL3PlusPlugin(nullptr),
            mLaparoscopeCamerasNode( nullptr ),
            mLaparoscopeCameraNode{ nullptr, nullptr },
            mVRCamerasNode( nullptr ),
            mVRCameraNode{ nullptr, nullptr },
            mLaparoscopeCameras{nullptr, nullptr },
            mZoom( 1.0f ),
            mCamNear( 0.005f ),
            mCamFar( 200.0f ),
            mLaparoscopeWorkspaces{ nullptr, nullptr },
            mVRWorkspaces{ nullptr, nullptr },
            mMirrorWorkspace( nullptr ),
            mVRCullCamera( nullptr ),
            mVRTexture( nullptr ),
            mVideoTexture{ nullptr, nullptr },
            mVrData(),
            mOvrCompositorListener( nullptr ),
            mHMD( nullptr ),
            mHmdConfig(esvr2->mConfig->hmdConfig),
            mVRCompositor( nullptr ),
            mStrDriver( "" ),
            mStrDisplay( "" ),
            mDeviceModelNumber( "" ),
            mStagingTexture{ nullptr, nullptr },
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 ),
            mLastFrameUpdate(0),
            mUpdateFrames(2),
            mQuit(false),
            mUseHlmsDiskCache(true),
            mUseMicrocodeCache(true)
//TODO: Load Resource Folder, Load Plugin Folder
//        GraphicsSystem( gameState, showOgreConfigDialog,
//                        RESOURCE_FOLDER, PLUGIN_FOLDER ),
    {
        LOG << "RESOURCE_FOLDER:" << RESOURCE_FOLDER << LOGEND;
        LOG << "PLUGIN_FOLDER:" << PLUGIN_FOLDER << LOGEND;
        const Ogre::Matrix4 id[2] =
                { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY };
        mVrData.set(id, id);
        memset( mTrackedDevicePose, 0, sizeof (mTrackedDevicePose) );
    }

    GraphicsSystem::~GraphicsSystem() {}

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

        mGL3PlusPlugin = OGRE_NEW Ogre::GL3PlusPlugin();
        mRoot->installPlugin( mGL3PlusPlugin );

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

        if( mEsvr2->mConfig->showOgreDialog || !mRoot->restoreConfig() )
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

        params.insert( std::make_pair("title", windowTitle) );
        params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
        params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
        params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
        params.insert( std::make_pair("reverse_depth", "Yes" ) );

        mRenderWindow = Ogre::Root::getSingleton().createRenderWindow(
                windowTitle, width, height, fullscreen, &params );

        mOverlaySystem = OGRE_NEW Ogre::v1::OverlaySystem();

        setupResources();
        loadResources();
        chooseSceneManager();
        createVRCameras();
        createLaparoscopeCameras();
        setupVRCompositor();
        setupLaparoscopeCompositors();

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

        if( mVRTexture )
        {
            textureManager->destroyTexture( mVRTexture );
            mVRTexture = 0;
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

            if( mLaparoscopeCameras[eye] )
            {
                mLaparoscopeSceneManager->destroyCamera(mLaparoscopeCameras[eye] );
                mLaparoscopeCameras[eye] = nullptr;
            }
        }

        if( mVRCullCamera )
        {
            mLaparoscopeSceneManager->destroyCamera( mVRCullCamera );
            mVRCullCamera = 0;
        }

        if( mHMD )
        {
            vr::VR_Shutdown();
            mHMD = NULL;
        }

        //TODO: deinitialize
//        GraphicsSystem::deinitialize();
    }

    void GraphicsSystem::setDistortion(Distortion dist)
    {
        if( mLaparoscopeWorkspaces[LEFT] && mLaparoscopeWorkspaces[RIGHT] )
        {
            for(size_t eye = 0; eye < mEyeNum; eye++)
            {
                mLaparoscopeWorkspaces[eye]->setViewportModifier(getVpOffset(dist, eye));
            }
        }
        mEsvr2->mVideoLoader->setDistortion(dist);
    }

    Ogre::Real GraphicsSystem::getZoom()
    {
        return mZoom;
    }

    void GraphicsSystem::setZoom( Ogre::Real zoom )
    {
        if( zoom > 0.1 && zoom < 10.0f )
            mZoom = zoom;
        Distortion dist = mEsvr2->mVideoLoader->getDistortion();
        if( mLaparoscopeWorkspaces[LEFT] && mLaparoscopeWorkspaces[RIGHT] )
        {
            for(size_t eye = 0; eye < mEyeNum; eye++)
            {
                mLaparoscopeWorkspaces[eye]->setViewportModifier(getVpOffset(dist, eye));
            }
        }
    }

    //-------------------------------------------------------------------------------
    void GraphicsSystem::createVRCameras()
    {
        //Use one node to control both cameras
//         mCamerasNode->setOrientation( orientation );
        mVRCamerasNode = mVRSceneManager
                ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
                ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mVRCamerasNode->setName( "Cameras Node" );
//         mCamerasNode->setPosition( pos );
//         mCamerasNode->setOrientation( orientation );

        mVRCameraNode[LEFT] = mVRCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mVRCameraNode[LEFT]->setName( "Left Camera Node" );

        mVRCameraNode[RIGHT] = mVRCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mVRCameraNode[RIGHT]->setName( "Right Camera Node" );

        mVRCullCamera = mVRSceneManager->createCamera( "VrCullCamera" );
        mVRCullCamera->detachFromParent();
        mVRCamerasNode->attachObject( mVRCullCamera );

//         mCamerasNode->setPosition(pos);

        //setup mVrData und mVrCullCamera using the mHmdConfig
        syncVRCameraProjection( true );

        if ( mEsvr2->mConfig->workspaceType == WS_TWO_CAMERAS_STEREO )
        {
            mVRCameras[LEFT] = mVRSceneManager->createCamera("Left Eye" );
            mVRCameras[RIGHT] = mVRSceneManager->createCamera("Right Eye" );

//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;

            //By default cameras are attached to the Root Scene Node.
            mVRCameras[LEFT]->detachFromParent();
            mVRCamerasNode->attachObject(mVRCameras[LEFT] );
            mVRCameras[RIGHT]->detachFromParent();
            mVRCamerasNode->attachObject(mVRCameras[RIGHT] );
//             mCameraNode[RIGHT]->attachObject( mEyeCameras[RIGHT] );
            //TODO: configure the cameras according to HMD config
        }
        if (mEsvr2->mConfig->workspaceType == WS_INSTANCED_STEREO)
        {
            mVRCameras[MONO] = mVRSceneManager->createCamera( "Main Camera" );

            // Position it at 500 in Z direction
            mVRCameras[MONO]->setPosition( Ogre::Vector3( 0.0, 0.0, 0.0 ) );
            // Look back along -Z
            mVRCameras[MONO]->lookAt( Ogre::Vector3( 0, 0, -1.0 ) );
            mVRCameras[MONO]->setNearClipDistance( mCamNear );
            mVRCameras[MONO]->setFarClipDistance( mCamFar );
            mVRCameras[MONO]->setAutoAspectRatio( true );
            mVRCameras[MONO]->detachFromParent();
            mVRCameras[MONO]->setVrData( &mVrData );
            mVRCamerasNode->attachObject( mVRCameras[MONO] );
        }

    }


    //-------------------------------------------------------------------------------
    void GraphicsSystem::createLaparoscopeCameras(void)
    {
        //Use one node to control both cameras
//         mCamerasNode->setOrientation( orientation );
        mLaparoscopeCamerasNode = mLaparoscopeSceneManager
                ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
                ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeCamerasNode->setName( "Cameras Node" );
//         mCamerasNode->setPosition( pos );
//         mCamerasNode->setOrientation( orientation );

        mLaparoscopeCameraNode[LEFT] = mVRCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeCameraNode[LEFT]->setName( "Left Camera Node" );

        mLaparoscopeCameraNode[RIGHT] = mLaparoscopeCamerasNode->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeCameraNode[RIGHT]->setName( "Right Camera Node" );
        mLaparoscopeCameras[LEFT] = mLaparoscopeSceneManager->createCamera("Left Eye" );
        mLaparoscopeCameras[RIGHT] = mLaparoscopeSceneManager->createCamera("Right Eye" );

//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;

        //By default cameras are attached to the Root Scene Node.
        mLaparoscopeCameras[LEFT]->detachFromParent();
        mLaparoscopeCamerasNode->attachObject(mLaparoscopeCameras[LEFT] );
        mLaparoscopeCameras[RIGHT]->detachFromParent();
        mLaparoscopeCamerasNode->attachObject(mLaparoscopeCameras[RIGHT] );
        //TODO: config cameras according to camera config
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
    void GraphicsSystem::syncVRCameraProjection( bool bForceUpdate )
    {
        if( bForceUpdate )
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
//             mVrData.set( eyeToHead, projectionMatrixRS );
            mVrData.set( eyeToHead, projectionMatrixRS );

            Ogre::Vector4 cameraCullFrustumExtents;
            cameraCullFrustumExtents.x = std::min(
                eyeFrustumExtents[0].x, eyeFrustumExtents[1].x );
            cameraCullFrustumExtents.y = std::max(
                eyeFrustumExtents[0].y, eyeFrustumExtents[1].y );
            cameraCullFrustumExtents.z = std::max(
                eyeFrustumExtents[0].z, eyeFrustumExtents[1].z );
            cameraCullFrustumExtents.w = std::min(
                eyeFrustumExtents[0].w, eyeFrustumExtents[1].w );

            mVRCullCamera->setFrustumExtents(
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
            mVRCullCamera->setNearClipDistance( mCamNear + offset );
            mVRCullCamera->setFarClipDistance( mCamFar + offset );
        }
    }

    Ogre::Vector4 GraphicsSystem::getVpOffset( Distortion dist, size_t eye)
    {
        size_t vr_width_half = mVRTexture->getWidth()/2;
        size_t vr_width = mVRTexture->getWidth();
        size_t vr_height = mVRTexture->getHeight();
        float f_x, f_y, c_x, c_y;
        StereoCameraConfig cameraConfig = mEsvr2->mVideoLoader->getStereoCameraConfig();
        switch(dist)
        {
            case DIST_RAW:
                f_x = cameraConfig.cfg[eye]->K[0] * mZoom;
                f_y = cameraConfig.cfg[eye]->K[4] * mZoom;
                c_x = cameraConfig.cfg[eye]->K[2];
                c_y = cameraConfig.cfg[eye]->K[5];
                break;
            case DIST_UNDISTORT_RECTIFY:
                f_x = cameraConfig.cfg[eye]->P[0] * mZoom;
                f_y = cameraConfig.cfg[eye]->P[5] * mZoom;
                c_x = cameraConfig.cfg[eye]->P[2];
                c_y = cameraConfig.cfg[eye]->P[6];
                break;
            default:
                return Ogre::Vector4(0,0,0,0);
        }
        float img_width = cameraConfig.cfg[eye]->width;
        float img_height = cameraConfig.cfg[eye]->height;

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
        Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();
        Ogre::TextureGpuManager *textureManager =
                mRoot->getRenderSystem()->getTextureGpuManager();

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
        size_t vr_width_half = mVRTexture->getWidth()/2;
        size_t vr_width = mVRTexture->getWidth();
        size_t vr_height = mVRTexture->getHeight();

        StereoCameraConfig cameraConfig =
                mEsvr2->mVideoLoader->getStereoCameraConfig();
        for (size_t eye = 0; eye < mEyeNum; eye++)
        {
            float far_plane = 100.0;
            float near_plane = 0.001;
            float img_width = cameraConfig.cfg[eye]->width;
            float img_height = cameraConfig.cfg[eye]->height;
            float f_x = cameraConfig.cfg[eye]->P[0] * mZoom;
            float f_y = cameraConfig.cfg[eye]->P[5] * mZoom;
            float c_x = cameraConfig.cfg[eye]->P[2];
            float c_y = cameraConfig.cfg[eye]->P[6];
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
            eyeToHead[eye][0][3] = cameraConfig.cfg[eye]->P[3] *2.0f / cameraConfig.cfg[eye]->P[0];
//                 mEyeCameras[eye]->setPosition(position );
//                 Ogre::Vector3 focusPoint = Ogre::Vector3( 0.0f, 0.0f, -1.0f );
//                 mCameraNode[eye]->lookAt( focusPoint, Ogre::Node::TS_LOCAL );
            Ogre::Matrix4 proj_matrix_rs;
            mRoot->getRenderSystem()->_convertOpenVrProjectionMatrix(
                proj_matrix[eye], proj_matrix_rs );

            mVRCameras[eye]->setCustomProjectionMatrix(true, proj_matrix[eye] );
//                 mEyeCameras[eye]->setAutoAspectRatio( true );
            mVRCameras[eye]->setNearClipDistance(0.0001 );
            mVRCameras[eye]->setFarClipDistance(mCamFar );
            double tx = (cameraConfig.cfg[eye]->P[3] / f_x);
            Ogre::Vector3 position = Ogre::Vector3::UNIT_X * tx;
            LOG << "Pos "<< cameraConfig.cfg[eye]->eye_str << eye << ": " << position.x << " " << position.y << " " <<position.z <<LOGEND;
            mVRCameras[eye]->setPosition(position );
            mVRCameras[eye]->lookAt(0, 0, 1);
        }
        // now as we have camera config we use it.
        mVrData.set( eyeToHead, proj_matrix);
        return true;
    }

    void GraphicsSystem::setupVRCompositor(void)
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        initOpenVR();
        calcAlign();

        //create Workspaces
        if ( mEsvr2->mConfig->workspaceType == WS_TWO_CAMERAS_STEREO )
        {
            createTwoWorkspaces();
        }
        else if (mEsvr2->mConfig->workspaceType == WS_INSTANCED_STEREO)
        {
            mVRWorkspaces[MONO] = compositorManager->addWorkspace(
                mVRSceneManager, mVRTexture,
                mVRCameras[MONO],
                "InstancedStereoWorkspace", true, 1 );
        }

        mOvrCompositorListener =
            new OpenVRCompositorListener(
                mHMD, mVRCompositor, mVRTexture,
                mRoot, mLaparoscopeWorkspaces,
                mVRCamerasNode, mEsvr2->mPoseState
            );
        setupImageData();

        syncVRCameraProjection( true );

        const Ogre::IdString workspaceName( "StereoMirrorWindowWorkspace" );

        Ogre::CompositorChannelVec channels( 2u );
        channels[0] = mRenderWindow->getTexture();
        channels[1] = mVRTexture;
        mMirrorWorkspace = compositorManager->addWorkspace(
            mVRSceneManager, channels, mVRCameras[LEFT],
            workspaceName, true, 3);
    }

    void GraphicsSystem::setupLaparoscopeCompositors(void)
    {
        Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();
        Ogre::TextureGpuManager *textureManager =
                mRoot->getRenderSystem()->getTextureGpuManager();
        int width = 960;
        int height = 540;
        std::string textureNames[2] = {"LaparoscopeViewLeft", "LaparoscopeViewRight" };
        for (size_t eye = 0; eye < mEyeNum; eye++ )
        {
            //Radial Density Mask requires the VR texture to be UAV & reinterpretable
            mLaparoscopeViewTexture[eye] = textureManager->createOrRetrieveTexture(
                    textureNames[eye],
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::RenderToTexture|
                    Ogre::TextureFlags::Uav|
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D );
            mLaparoscopeViewTexture[eye]->setResolution(width, height);
            mLaparoscopeViewTexture[eye]->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
            mLaparoscopeViewTexture[eye]->scheduleTransitionTo(
                    Ogre::GpuResidency::Resident );
            Ogre::CompositorManager2 *compositorManager =
                    mRoot->getCompositorManager2();
            mLaparoscopeWorkspaces[eye] = compositorManager->addWorkspace(
                    mLaparoscopeSceneManager,
                    mLaparoscopeViewTexture[eye],
                    mLaparoscopeCameras[eye],
                    "LaparoscopeViewWorkspace",
                    true,
                    0);
        }
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
        mVRTexture = textureManager->createOrRetrieveTexture(
            "OpenVR Both Eyes",
            Ogre::GpuPageOutStrategy::Discard,
            Ogre::TextureFlags::RenderToTexture|
            Ogre::TextureFlags::Uav|
            Ogre::TextureFlags::Reinterpretable,
            Ogre::TextureTypes::Type2D );
        new_width = width << 1u;
        mVRTexture->setResolution( new_width, height);
        mVRTexture->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
        mVRTexture->scheduleTransitionTo(
            Ogre::GpuResidency::Resident );


    }

    void GraphicsSystem::createTwoWorkspaces()
    {
        Ogre::CompositorManager2 *compositorManager =
            mRoot->getCompositorManager2();
        const Ogre::IdString workspaceName( "StereoRenderingWorkspace" );
        Ogre::uint8 vpModifierMask, executionMask;
        Ogre::Vector4 vpOffsetScale;

        vpModifierMask  = 0x01;
        executionMask   = 0x01;
        //set offset so that we only render to the portion of the screen where there is the image
        vpOffsetScale   = Ogre::Vector4( 0.0f,	 0.0f, 0.5f, 1.0f );
        mVRWorkspaces[LEFT] = compositorManager->addWorkspace(
                mLaparoscopeSceneManager,
                mVRTexture,
                mLaparoscopeCameras[LEFT], workspaceName,
                true, 1, nullptr,
                nullptr,
                vpOffsetScale,
                vpModifierMask,
                executionMask );

        vpModifierMask  = 0x02;
        executionMask   = 0x02;
        vpOffsetScale   = Ogre::Vector4( 0.5f, 0.0f, 0.5f, 1.0f );
        mVRWorkspaces[RIGHT] = compositorManager->addWorkspace(
                mLaparoscopeSceneManager,
                mVRTexture,
                mLaparoscopeCameras[RIGHT], workspaceName,
                true, 1, nullptr,
                nullptr,
                vpOffsetScale,
                vpModifierMask,
                executionMask);
    }

    void GraphicsSystem::setupImageData()
    {
        if (!mLaparoscopeWorkspaces[LEFT])
            return;
        Ogre::TextureGpuManager *textureManager =
            mRoot->getRenderSystem()->getTextureGpuManager();

        if ( mEsvr2->mConfig->videoRenderTarget == VRT_TO_2D_RECTANGLE )
        {
            if ( mEsvr2->mConfig->isStereo )
            {
                //TODO: guard if we don't find it.
                LOG << "setup Video Texture Left" << LOGEND;
                mVideoTexture[LEFT] = mLaparoscopeWorkspaces[LEFT]->findNode("TwoCamerasNode")->getLocalTextures()[0];
                mVideoTexture[RIGHT] = mLaparoscopeWorkspaces[RIGHT]->findNode("TwoCamerasNode")->getLocalTextures()[1];
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
                !mEsvr2->mVideoLoader->updateDestinationSize(
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

    void GraphicsSystem::beginFrameParallel(void)
    {
        if ( mOvrCompositorListener->getFrameCnt() >=
                mLastFrameUpdate + mUpdateFrames &&
            mVideoTexture && mStagingTexture[LEFT])
        {
            mLastFrameUpdate = mOvrCompositorListener->getFrameCnt();
//             LOG << "update" << LOGEND;
            StereoImageData imgData = mEsvr2->mVideoLoader->getCurStereoImageData();
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
//            LOG << "wait" << LOGEND;
        }
        if ( mEsvr2->mPoseState && mEsvr2->mPoseState->validPose() )
        {
            Ogre::Quaternion prev_orientation =
                mLaparoscopeCamerasNode->getOrientation();
            Ogre::Vector3 prev_position = mLaparoscopeCamerasNode->getPosition();
            Ogre::Quaternion orientation =
                mEsvr2->mPoseState->getOrientation();
            orientation = orientation * Ogre::Quaternion(
                Ogre::Degree(180), Ogre::Vector3::UNIT_Z);
            Ogre::Vector3 position = mEsvr2->mPoseState->getPosition();
            if ( !prev_orientation.orientationEquals(orientation)
                || prev_position != position )
            {
//                 LOG << "update orientation and position" << mOvrCompositorListener->getFrameCnt();
//                 LOG << "Pos: " << position.x << " " << position.y << " " <<position.z;
//                 LOG << "Orientation: " << orientation.w << " "<< orientation.x << " " << orientation.y << " " <<orientation.z<<LOGEND;
                mLaparoscopeCamerasNode->setPosition( position );
                mLaparoscopeCamerasNode->setOrientation( orientation );
            }
        }
    }

    //-----------------------------------------------------------------------------------
    void GraphicsSystem::setupResources(void)
    {
        // Load resource paths from config file
        Ogre::ConfigFile cf;
        cf.load(mResourcePath + "Resources.cfg");

        // Go through all sections & settings in the file
        Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

        Ogre::String secName, typeName, archName;
        while( seci.hasMoreElements() )
        {
            secName = seci.peekNextKey();
            Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();

            if( secName != "Hlms" )
            {
                Ogre::ConfigFile::SettingsMultiMap::iterator i;
                for (i = settings->begin(); i != settings->end(); ++i)
                {
                    typeName = i->first;
                    archName = i->second;
                    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                            archName, typeName, secName);
                }
            }
        }
    }

    //-----------------------------------------------------------------------------------
    void GraphicsSystem::loadResources(void)
    {
        registerHlms();

        loadTextureCache();
        loadHlmsDiskCache();

        // Initialise, parse scripts etc
        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups( true );
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
        size_t numWorkspaces = compositorManager->getNumWorkspaces();
        LOG << numWorkspaces << LOGEND;
        // Initialize resources for LTC area lights and accurate specular reflections (IBL)
        Ogre::Hlms *hlms = mRoot->getHlmsManager()->getHlms( Ogre::HLMS_PBS );
        OGRE_ASSERT_HIGH( dynamic_cast<Ogre::HlmsPbs*>( hlms ) );
        Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlms );
        try
        {
            hlmsPbs->loadLtcMatrix();
        }
        catch( Ogre::FileNotFoundException &e )
        {
            Ogre::LogManager::getSingleton().logMessage( e.getFullDescription(), Ogre::LML_CRITICAL );
            Ogre::LogManager::getSingleton().logMessage(
                    "WARNING: LTC matrix textures could not be loaded. Accurate specular IBL reflections "
                    "and LTC area lights won't be available or may not function properly!",
                    Ogre::LML_CRITICAL );
        }
    }

    //-----------------------------------------------------------------------------------
    void GraphicsSystem::registerHlms(void)
    {
        Ogre::ConfigFile cf;
        cf.load( mResourcePath + "Resources.cfg" );

#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE || OGRE_PLATFORM == OGRE_PLATFORM_APPLE_IOS
        Ogre::String rootHlmsFolder = Ogre::macBundlePath() + '/' +
                                  cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
#else
        Ogre::String rootHlmsFolder = cf.getSetting( "DoNotUseAsResource", "Hlms", "" );
        if ( *(rootHlmsFolder.begin()) != '/' )
            rootHlmsFolder = mResourcePath + rootHlmsFolder;
#endif

        if( rootHlmsFolder.empty() )
            rootHlmsFolder = "./";
        else if( *(rootHlmsFolder.end() - 1) != '/' )
            rootHlmsFolder += "/";

        //At this point rootHlmsFolder should be a valid path to the Hlms data folder

        Ogre::HlmsUnlit *hlmsUnlit = 0;
        Ogre::HlmsPbs *hlmsPbs = 0;

        //For retrieval of the paths to the different folders needed
        Ogre::String mainFolderPath;
        Ogre::StringVector libraryFoldersPaths;
        Ogre::StringVector::const_iterator libraryFolderPathIt;
        Ogre::StringVector::const_iterator libraryFolderPathEn;

        Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

        {
            //Create & Register HlmsUnlit
            //Get the path to all the subdirectories used by HlmsUnlit
            Ogre::HlmsUnlit::getDefaultPaths( mainFolderPath, libraryFoldersPaths );
            Ogre::Archive *archiveUnlit = archiveManager.load( rootHlmsFolder + mainFolderPath,
                                                               "FileSystem", true );
            Ogre::ArchiveVec archiveUnlitLibraryFolders;
            libraryFolderPathIt = libraryFoldersPaths.begin();
            libraryFolderPathEn = libraryFoldersPaths.end();
            while( libraryFolderPathIt != libraryFolderPathEn )
            {
                Ogre::Archive *archiveLibrary =
                        archiveManager.load( rootHlmsFolder + *libraryFolderPathIt, "FileSystem", true );
                archiveUnlitLibraryFolders.push_back( archiveLibrary );
                ++libraryFolderPathIt;
            }

            //Create and register the unlit Hlms
            hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit( archiveUnlit, &archiveUnlitLibraryFolders );
            Ogre::Root::getSingleton().getHlmsManager()->registerHlms( hlmsUnlit );
        }

        {
            //Create & Register HlmsPbs
            //Do the same for HlmsPbs:
            Ogre::HlmsPbs::getDefaultPaths( mainFolderPath, libraryFoldersPaths );
            Ogre::Archive *archivePbs = archiveManager.load( rootHlmsFolder + mainFolderPath,
                                                             "FileSystem", true );

            //Get the library archive(s)
            Ogre::ArchiveVec archivePbsLibraryFolders;
            libraryFolderPathIt = libraryFoldersPaths.begin();
            libraryFolderPathEn = libraryFoldersPaths.end();
            while( libraryFolderPathIt != libraryFolderPathEn )
            {
                Ogre::Archive *archiveLibrary =
                        archiveManager.load( rootHlmsFolder + *libraryFolderPathIt, "FileSystem", true );
                archivePbsLibraryFolders.push_back( archiveLibrary );
                ++libraryFolderPathIt;
            }

            //Create and register
            hlmsPbs = OGRE_NEW Ogre::HlmsPbs( archivePbs, &archivePbsLibraryFolders );
            Ogre::Root::getSingleton().getHlmsManager()->registerHlms( hlmsPbs );
        }


        Ogre::RenderSystem *renderSystem = mRoot->getRenderSystem();
        if( renderSystem->getName() == "Direct3D11 Rendering Subsystem" )
        {
            //Set lower limits 512kb instead of the default 4MB per Hlms in D3D 11.0
            //and below to avoid saturating AMD's discard limit (8MB) or
            //saturate the PCIE bus in some low end machines.
            bool supportsNoOverwriteOnTextureBuffers;
            renderSystem->getCustomAttribute( "MapNoOverwriteOnDynamicBufferSRV",
                                              &supportsNoOverwriteOnTextureBuffers );

            if( !supportsNoOverwriteOnTextureBuffers )
            {
                hlmsPbs->setTextureBufferDefaultSize( 512 * 1024 );
                hlmsUnlit->setTextureBufferDefaultSize( 512 * 1024 );
            }
        }
    }
    //-----------------------------------------------------------------------------------
    void GraphicsSystem::loadTextureCache(void)
    {
#if !OGRE_NO_JSON
        Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();
        Ogre::Archive *rwAccessFolderArchive = archiveManager.load( mWriteAccessFolder,
                                                                    "FileSystem", true );
        try
        {
            const Ogre::String filename = "textureMetadataCache.json";
            if( rwAccessFolderArchive->exists( filename ) )
            {
                Ogre::DataStreamPtr stream = rwAccessFolderArchive->open( filename );
                std::vector<char> fileData;
                fileData.resize( stream->size() + 1 );
                if( !fileData.empty() )
                {
                    stream->read( &fileData[0], stream->size() );
                    //Add null terminator just in case (to prevent bad input)
                    fileData.back() = '\0';
                    Ogre::TextureGpuManager *textureManager =
                            mRoot->getRenderSystem()->getTextureGpuManager();
                    textureManager->importTextureMetadataCache( stream->getName(), &fileData[0], false );
                }
            }
            else
            {
                Ogre::LogManager::getSingleton().logMessage(
                        "[INFO] Texture cache not found at " + mWriteAccessFolder +
                        "/textureMetadataCache.json" );
            }
        }
        catch( Ogre::Exception &e )
        {
            Ogre::LogManager::getSingleton().logMessage( e.getFullDescription() );
        }

        archiveManager.unload( rwAccessFolderArchive );
#endif
    }
    //-----------------------------------------------------------------------------------
    void GraphicsSystem::loadHlmsDiskCache(void)
    {
        if( !mUseMicrocodeCache && !mUseHlmsDiskCache )
            return;

        Ogre::HlmsManager *hlmsManager = mRoot->getHlmsManager();
        Ogre::HlmsDiskCache diskCache( hlmsManager );

        Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

        Ogre::Archive *rwAccessFolderArchive = archiveManager.load( mWriteAccessFolder,
                                                                    "FileSystem", true );

        if( mUseMicrocodeCache )
        {
            //Make sure the microcode cache is enabled.
            Ogre::GpuProgramManager::getSingleton().setSaveMicrocodesToCache( true );
            const Ogre::String filename = "microcodeCodeCache.cache";
            if( rwAccessFolderArchive->exists( filename ) )
            {
                Ogre::DataStreamPtr shaderCacheFile = rwAccessFolderArchive->open( filename );
                Ogre::GpuProgramManager::getSingleton().loadMicrocodeCache( shaderCacheFile );
            }
        }

        if( mUseHlmsDiskCache )
        {
            for( size_t i=Ogre::HLMS_LOW_LEVEL + 1u; i<Ogre::HLMS_MAX; ++i )
            {
                Ogre::Hlms *hlms = hlmsManager->getHlms( static_cast<Ogre::HlmsTypes>( i ) );
                if( hlms )
                {
                    Ogre::String filename = "hlmsDiskCache" +
                                            Ogre::StringConverter::toString( i ) + ".bin";

                    try
                    {
                        if( rwAccessFolderArchive->exists( filename ) )
                        {
                            Ogre::DataStreamPtr diskCacheFile = rwAccessFolderArchive->open( filename );
                            diskCache.loadFrom( diskCacheFile );
                            diskCache.applyTo( hlms );
                        }
                    }
                    catch( Ogre::Exception& )
                    {
                        Ogre::LogManager::getSingleton().logMessage(
                                "Error loading cache from " + mWriteAccessFolder + "/" +
                                filename + "! If you have issues, try deleting the file "
                                           "and restarting the app" );
                    }
                }
            }
        }

        archiveManager.unload( mWriteAccessFolder );
    }

    //-----------------------------------------------------------------------------------
    void GraphicsSystem::chooseSceneManager(void)
    {
#if OGRE_DEBUG_MODE
        //Debugging multithreaded code is a PITA, disable it.
        const size_t numThreads = 1;
#else
        //getNumLogicalCores() may return 0 if couldn't detect
        const size_t numThreads = std::max<size_t>( 1, Ogre::PlatformInformation::getNumLogicalCores() );
#endif
        // Create the SceneManager, in this case a generic one
        mLaparoscopeSceneManager = mRoot->createSceneManager( Ogre::ST_GENERIC,
                                                   numThreads,
                                                   "LaparoscopeScene" );
        mVRSceneManager = mRoot->createSceneManager( Ogre::ST_GENERIC,
                                                   numThreads,
                                                   "VRView" );

        mVRSceneManager->addRenderQueueListener( mOverlaySystem );
        mVRSceneManager->getRenderQueue()->setSortRenderQueue(
                Ogre::v1::OverlayManager::getSingleton().mDefaultRenderQueueId,
                Ogre::RenderQueue::StableSort );

        //Set sane defaults for proper shadow mapping
        mVRSceneManager->setShadowDirectionalLightExtrusionDistance( 500.0f );
        mVRSceneManager->setShadowFarDistance( 500.0f );
    }

    void GraphicsSystem::setQuit()
    {
        mQuit = true;
    }

    bool GraphicsSystem::getQuit()
    {
        return mQuit;
    }

    Ogre::Window *GraphicsSystem::getRenderWindow()
    {
        return mRenderWindow;
    }

    void GraphicsSystem::update( float timesincelast)
    {
        Ogre::WindowEventUtilities::messagePump();
        if( mRenderWindow->isVisible() )
            mQuit |= !mRoot->renderOneFrame();

        //THis is for counting when we heard sth from logic system
//        mAccumTimeSinceLastLogicFrame += timeSinceLast;

    }
}
