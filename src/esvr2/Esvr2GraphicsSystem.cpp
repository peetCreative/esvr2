#include "Esvr2GraphicsSystem.h"

#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2OpenVRCompositorListener.h"
#include "Esvr2Helper.h"

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
#include <Overlay/OgreOverlayManager.h>

#include "Hlms/Unlit/OgreHlmsUnlit.h"
#include "Hlms/Pbs/OgreHlmsPbs.h"

#include "opencv2/opencv.hpp"
#include <sstream>
#include <cmath>
#include <mutex>
#include <vector>

namespace esvr2
{
    GraphicsSystem::GraphicsSystem(
            Esvr2 *esvr2):
            mEsvr2(esvr2),
            mGameState(nullptr),
            mOvrCompositorListener( nullptr ),
            mRoot(nullptr),
            mGL3PlusPlugin(nullptr),
            mWindow(nullptr),
            mWindowTitle("Esvr2"),
            mPluginsFolder(),
            mWriteAccessFolder(),
            mUseHlmsDiskCache(true),
            mUseMicrocodeCache(true),
            mLaparoscopeWorkspaces{ nullptr, nullptr },
            mVRWorkspaces{ nullptr, nullptr },
            mMirrorWorkspace( nullptr ),
            mLaparoscopeSceneManager(nullptr),
            mVRSceneManager(nullptr),
            mLaparoscopeCameras{nullptr, nullptr },
            mVRCameraNear(0.3),
            mVRCameraFar(30.0),
            mVRCameras{nullptr, nullptr },
            mVRCullCamera( nullptr ),
            mVRTexture( nullptr ),
            mVideoTexture{nullptr, nullptr },
            mStagingTextures{nullptr, nullptr },
            mDebugWindow(nullptr),
            mDebugWS(nullptr),
            mDebugCamera(nullptr),
            mDebugCameraNode(nullptr),
            mQuit(false),
            mShowVideo(true),
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 ),
            mLastFrameUpdate(0),
            mVideoUpdateFrames(0)
    {
        const Ogre::Matrix4 id[2] =
                { Ogre::Matrix4::IDENTITY, Ogre::Matrix4::IDENTITY };
        mVrData.set(id, id);
        //TODO: Load Resource Folder, Load Plugin Folder
        LOG << "RESOURCE_FOLDER:" << RESOURCE_FOLDER << LOGEND;
        LOG << "PLUGIN_FOLDER:" << PLUGIN_FOLDER << LOGEND;
    }

    //-----------------------------------------------------------------------------------
    void GraphicsSystem::setupResources(void)
    {
        // Load resource paths from config file
        Ogre::ConfigFile cf;
        cf.load( mEsvr2->mConfig->resourcePath );

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
        cf.load( mEsvr2->mConfig->resourcePath );

        Ogre::String rootHlmsFolder = cf.getSetting( "DoNotUseAsResource", "Hlms", "" );

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
            mRoot->getHlmsManager()->registerHlms( hlmsUnlit );
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


        //Set sane defaults for proper shadow mapping
        mVRSceneManager->setShadowDirectionalLightExtrusionDistance( 500.0f );
        mVRSceneManager->setShadowFarDistance( 500.0f );
    }

    //-------------------------------------------------------------------------------
    void GraphicsSystem::createVRCameras()
    {
        //Use one node to control both cameras
//         mCamerasNode->setOrientation( orientation );
//         mCamerasNode->setOrientation( orientation );

        mVRCullCamera = mVRSceneManager->createCamera( "VrCullCamera" );

//         mCamerasNode->setPosition(pos);

//        if ( mEsvr2->mConfig->workspaceType == WS_TWO_CAMERAS_STEREO )
//        {
        std::string cameraNames[2] = {"VR Left Eye", "VR Right Eye"};
        for(size_t eye = 0; eye < mEyeNum; eye++)
        {
            mVRCameras[eye] = mVRSceneManager->createCamera(cameraNames[eye]);
            mVRCameras[eye]->setFarClipDistance(mVRCameraFar);
            mVRCameras[eye]->setNearClipDistance(mVRCameraNear);
            mVRCameras[eye]->lookAt(0,0,-1);
            HmdConfig hmdConfig = mEsvr2->mConfig->hmdConfig;
            if(hmdConfig.valid())
            {
                RealVector *vec = hmdConfig.projectionMatrixPtr[eye];
                Ogre::Matrix4 projMat = RealVectorToMatrix4(*vec);
                mVRCameras[eye]->setCustomProjectionMatrix(true, projMat);
                RealVector *vec1 = hmdConfig.eyeToHeadPtr[eye];
                Ogre::Vector3 pose = RealVectorToVector3(*vec1);
                mVRCameras[eye]->setPosition(pose);
            }
        }

        if(mOvrCompositorListener)
        {
            mOvrCompositorListener->syncVRCameraProjection( true );
        }
//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;
//        }
    }

    //-------------------------------------------------------------------------------
    void GraphicsSystem::createLaparoscopeCameras(void)
    {
        //Use one node to control both cameras
        mLaparoscopeCameras[LEFT] = mLaparoscopeSceneManager->createCamera("Lap Left Eye" );
        mLaparoscopeCameras[RIGHT] = mLaparoscopeSceneManager->createCamera("Lap Right Eye" );
        mLaparoscopeCameras[LEFT]->setFarClipDistance(30);
        mLaparoscopeCameras[RIGHT]->setFarClipDistance(30);
        mLaparoscopeCameras[LEFT]->setNearClipDistance(0.3);
        mLaparoscopeCameras[RIGHT]->setNearClipDistance(0.3);

//             const Ogre::Real eyeDistance        = 0.06f;
//             const Ogre::Real eyeFocusDistance   = 0.06f;

        //By default cameras are attached to the Root Scene Node.
        //TODO: config cameras according to camera config
    }

    //-----------------------------------------------------------------------------------
    // Just little bit modified code from GraphicsSystem
    void GraphicsSystem::initialize()
    {
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
                mQuit = true;
                return;
            }
        }

        mRoot->initialise( false, mWindowTitle );

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

        params.insert( std::make_pair("title", mWindowTitle) );
        params.insert( std::make_pair("gamma", cfgOpts["sRGB Gamma Conversion"].currentValue) );
        params.insert( std::make_pair("FSAA", cfgOpts["FSAA"].currentValue) );
        params.insert( std::make_pair("vsync", cfgOpts["VSync"].currentValue) );
        params.insert( std::make_pair("reverse_depth", "Yes" ) );

        if (mEsvr2->mConfig->debugScreen)
        {
            mDebugWindow = mRoot->createRenderWindow(
                    mWindowTitle, 500, 300, fullscreen, &params);

        }
        else
        {
            mWindow = mRoot->createRenderWindow(
                    mWindowTitle, width, height, fullscreen, &params );
        }

        //Resources
        setupResources();
        loadResources();

        //we need to create them here because SceneManager needs them
        setupLaparoscopeTextures();

        mOvrCompositorListener =
                new OpenVRCompositorListener(this);
        if (mOvrCompositorListener->initOpenVR())
        {
            setupVRTextures();
            mRoot->addFrameListener(mOvrCompositorListener);
        }
        else
        {
            delete mOvrCompositorListener;
            mOvrCompositorListener = nullptr;
        }

        Ogre::CompositorManager2 *compositorManager =
        mRoot->getCompositorManager2();
        // for some reason we can only update every n frames.
        //which is limited by this function
        mVideoUpdateFrames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();

        //create Scene
        chooseSceneManager();
        createVRCameras();
        createLaparoscopeCameras();
        //create Gamestate
        mGameState = new GameState(mEsvr2, this);
        mGameState->loadDatablocks();
        mGameState->createLaparoscopeScene();
        mGameState->createVRScene();

        if (mEsvr2->mConfig->debugScreen)
        {
            setupDebugScreen();
        }
        else
        {
            setupVRCompositor();
        }
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
            if( mStagingTextures[eye] )
            {
                //Tell the TextureGpuManager we're done with this StagingTexture. Otherwise it will leak.
                textureManager->removeStagingTexture(mStagingTextures[eye] );
                mStagingTextures[eye] = 0;
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

        //TODO: deinitialize
//        GraphicsSystem::deinitialize();
    }

    //configure Laparoscope Cameras according to CameraConfig
    bool GraphicsSystem::configureLaparoscopeCamera()
    {
//        Ogre::CompositorManager2 *compositorManager =
//                mRoot->getCompositorManager2();
//        // for some reason we can only update every n frames.
//        //which is limited by this function
//        mVideoUpdateFrames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();
//        StereoCameraConfig cameraConfig =
//                mEsvr2->mVideoLoader->getStereoCameraConfig();
//        for (size_t eye = 0; eye < mEyeNum; eye++)
//        {
//            float far_plane = 100.0;
//            float near_plane = 0.001;
//            float img_width = cameraConfig.cfg[eye]->width;
//            float img_height = cameraConfig.cfg[eye]->height;
//            float f_x = cameraConfig.cfg[eye]->P[0] * mZoom;
//            float f_y = cameraConfig.cfg[eye]->P[5] * mZoom;
//            float c_x = cameraConfig.cfg[eye]->P[2];
//            float c_y = cameraConfig.cfg[eye]->P[6];
//            float win_width = mHmdConfig.width;
//            float win_height = mHmdConfig.height;
//            float zoom_x = 1.0f;
//            float zoom_y = zoom_x;
//
//            // Preserve aspect ratio
////                 if (win_width != 0 && win_height != 0)
////                 {
////                     float img_aspect = (img_width / f_x) / (img_height / f_y);
////                     float win_aspect = win_width / win_height;
////
////                     if (img_aspect > win_aspect)
////                     {
////                         zoom_y = zoom_y / img_aspect * win_aspect;
////                     }
////                     else
////                     {
////                         zoom_x = zoom_x / win_aspect * img_aspect;
////                     }
////                 }
//
//            eyeToHead[eye] = Ogre::Matrix4::IDENTITY;
//            proj_matrix[eye] = Ogre::Matrix4::ZERO;
//
//            proj_matrix[eye][0][0] = 2.0 * f_x / img_width * zoom_x;
//            proj_matrix[eye][1][1] = 2.0 * f_y / img_height * zoom_y;
//
//            proj_matrix[eye][0][2] = 2.0 * (0.5 - c_x / img_width) * zoom_x;
//            proj_matrix[eye][1][2] = 2.0 * (c_y / img_height - 0.5) * zoom_y;
//
//            proj_matrix[eye][2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
//            proj_matrix[eye][2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);
//
//            proj_matrix[eye][3][2] = -1;
//            // we have also to set position and orientation
////                 mEyeCameras[eye]->setCustomProjectionMatrix( true, proj_matrix);
//            //2.0f ~= cameraConfig.cfg[eye].P[0]
//            // x   ~= cameraConfig.cfg[eye].P[3]
//            eyeToHead[eye][0][3] = cameraConfig.cfg[eye]->P[3] *2.0f / cameraConfig.cfg[eye]->P[0];
////                 mEyeCameras[eye]->setPosition(position );
////                 Ogre::Vector3 focusPoint = Ogre::Vector3( 0.0f, 0.0f, -1.0f );
////                 mCameraNode[eye]->lookAt( focusPoint, Ogre::Node::TS_LOCAL );
//            Ogre::Matrix4 proj_matrix_rs;
//            mRoot->getRenderSystem()->_convertOpenVrProjectionMatrix(
//                    proj_matrix[eye], proj_matrix_rs );
//
//            mVRCameras[eye]->setCustomProjectionMatrix(true, proj_matrix[eye] );
////                 mEyeCameras[eye]->setAutoAspectRatio( true );
//            mVRCameras[eye]->setNearClipDistance(0.0001 );
//            mVRCameras[eye]->setFarClipDistance(mCamFar );
//            double tx = (cameraConfig.cfg[eye]->P[3] / f_x);
//            Ogre::Vector3 position = Ogre::Vector3::UNIT_X * tx;
//            LOG << "Pos "<< cameraConfig.cfg[eye]->eye_str << eye << ": " << position.x << " " << position.y << " " <<position.z <<LOGEND;
//            mVRCameras[eye]->setPosition(position );
//            mVRCameras[eye]->lookAt(0, 0, 1);
//        }
//        // now as we have camera config we use it.
//        mVrData.set( eyeToHead, proj_matrix);
        return true;
    }

    //configure VR Cameras according to CameraConfig
    bool GraphicsSystem::configureVRCamera()
    {

        return true;
    }

    void GraphicsSystem::setupVRTextures(void)
    {
        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
        //Radial Density Mask requires the VR texture to be UAV & reinterpretable
        uint32_t new_width, width, height;
        mOvrCompositorListener->mHMD->GetRecommendedRenderTargetSize( &width, &height );
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

    void GraphicsSystem::setupVRCompositor(void)
    {
        Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();
        Ogre::TextureGpu *vrTexture;
        if(mOvrCompositorListener)
        {
             vrTexture = mVRTexture;
        }
        else
        {
            vrTexture = mWindow->getTexture();
        }

        //create Stereo Workspaces
        const Ogre::IdString workspaceNameVR("StereoRenderingWorkspace" );
        Ogre::uint8 vpModifierMask, executionMask;
        Ogre::Vector4 vpOffsetScale;

        vpModifierMask  = 0x01;
        executionMask   = 0x01;
        //set offset so that we only render to the portion of the screen where there is the image
        vpOffsetScale   = Ogre::Vector4( 0.0f,	 0.0f, 0.5f, 1.0f );
        mVRWorkspaces[LEFT] = compositorManager->addWorkspace(
                mVRSceneManager,
                vrTexture,
                mVRCameras[LEFT], workspaceNameVR,
                true, 1, nullptr,
                nullptr,
                vpOffsetScale,
                vpModifierMask,
                executionMask );

        vpModifierMask  = 0x02;
        executionMask   = 0x02;
        vpOffsetScale   = Ogre::Vector4( 0.5f, 0.0f, 0.5f, 1.0f );
        mVRWorkspaces[RIGHT] = compositorManager->addWorkspace(
                mVRSceneManager,
                vrTexture,
                mVRCameras[RIGHT], workspaceNameVR,
                true, 1,
                nullptr,
                nullptr,
                vpOffsetScale,
                vpModifierMask,
                executionMask);

        //if we have VR than we have to copy the texture
        if (mOvrCompositorListener)
        {
            //As we do not have instanced stereo we can only use the whole workspace
            mVRWorkspaces[RIGHT]->addListener(mOvrCompositorListener);

            const Ogre::IdString workspaceNameMirror( "StereoMirrorWindowWorkspace" );

            Ogre::CompositorChannelVec channels( 2u );
            channels[0] = mWindow->getTexture();
            channels[1] = mVRTexture;
            mMirrorWorkspace = compositorManager->addWorkspace(
                    mVRSceneManager, channels, mVRCameras[LEFT],
                    workspaceNameMirror, true, 3);
        }
    }

    void GraphicsSystem::setupLaparoscopeTextures()
    {
        Ogre::TextureGpuManager *textureManager =
                mRoot->getRenderSystem()->getTextureGpuManager();
        StereoCameraConfig cameraConfig =
                mEsvr2->mVideoLoader->getStereoCameraConfig();
        std::string viewTextureNames[2] = {"LaparoscopeViewLeft", "LaparoscopeViewRight" };
        std::string videoTextureNames[2] = {"VideoTextureLeft", "VideoTextureRight" };
        for (size_t eye = 0; eye < mEyeNum; eye++ )
        {
            //Radial Density Mask requires the VR texture to be UAV & reinterpretable
            mLaparoscopeViewTexture[eye] = textureManager->createOrRetrieveTexture(
                    viewTextureNames[eye],
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::RenderToTexture |
                    Ogre::TextureFlags::Uav |
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D);
            mLaparoscopeViewTexture[eye]->setResolution(
                    cameraConfig.cfg[eye]->width,
                    cameraConfig.cfg[eye]->height);
            mLaparoscopeViewTexture[eye]->setPixelFormat(Ogre::PFG_RGBA8_UNORM);
            mLaparoscopeViewTexture[eye]->scheduleTransitionTo(
                    Ogre::GpuResidency::Resident);
            //the video texture
            mVideoTexture[eye] = textureManager->createOrRetrieveTexture(
                    videoTextureNames[eye],
                    Ogre::GpuPageOutStrategy::Discard,
                    Ogre::TextureFlags::AutomaticBatching |
                    Ogre::TextureFlags::ManualTexture |
                    Ogre::TextureFlags::Reinterpretable,
                    Ogre::TextureTypes::Type2D );
            mVideoTexture[eye]->setResolution(
                    cameraConfig.cfg[eye]->width,
                    cameraConfig.cfg[eye]->height);
            mVideoTexture[eye]->setPixelFormat(Ogre::PFG_RGBA8_UNORM);
            mVideoTexture[eye]->scheduleTransitionTo(
                    Ogre::GpuResidency::Resident);
            //the staging texture to upload
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
                mQuit = true;
            }
            mStagingTextures[eye] =
                    textureManager->getStagingTexture(
                            mVideoTexture[eye]->getWidth(),
                            mVideoTexture[eye]->getHeight(),
                            mVideoTexture[eye]->getDepth(),
                            mVideoTexture[eye]->getNumSlices(),
                            mVideoTexture[eye]->getPixelFormat() );

        }
    }

    void GraphicsSystem::setupLaparoscopeCompositors(void)
    {
        Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();
        for (size_t eye = 0; eye < mEyeNum; eye++ )
        {
            Ogre::CompositorChannelVec channels( 2u );
            channels[0] = mLaparoscopeViewTexture[eye];
            channels[1] = mVideoTexture[eye];

            mLaparoscopeWorkspaces[eye] = compositorManager->addWorkspace(
                    mLaparoscopeSceneManager,
                    channels,
                    mLaparoscopeCameras[eye],
                    "LaparoscopeViewWorkspace",
                    true,
                    0);
        }
    }

    //Upload Videodata to GPU
    //Update Laparoscope Pose
    void GraphicsSystem::uploadVideoData2GPU(void)
    {
        if ( mFrameCnt >=
             mLastFrameUpdate + mVideoUpdateFrames &&
             mVideoTexture[LEFT] && mStagingTextures[LEFT] &&
             (mEyeNum < 2 || (mVideoTexture[RIGHT] && mStagingTextures[RIGHT])))
        {
            mLastFrameUpdate = mFrameCnt;
//             LOG << "update" << LOGEND;
            StereoImageData imgData = mEsvr2->mVideoLoader->getCurStereoImageData();
            for( size_t eye = 0; eye < mEyeNum; eye++ )
            {
                const size_t bytesPerRow =
                    mVideoTexture[eye]->_getSysRamCopyBytesPerRow(0 );

                mStagingTextures[eye]->startMapRegion();
                Ogre::TextureBox texBox = mStagingTextures[eye]->mapRegion(
                        mVideoTexture[eye]->getWidth(),
                        mVideoTexture[eye]->getHeight(),
                        mVideoTexture[eye]->getDepth(),
                        mVideoTexture[eye]->getNumSlices(),
                        mVideoTexture[eye]->getPixelFormat() );
                texBox.copyFrom(imgData.img[eye].data,
                                mVideoTexture[eye]->getWidth(),
                                mVideoTexture[eye]->getHeight(),
                                bytesPerRow );
                mStagingTextures[eye]->stopMapRegion();
                mStagingTextures[eye]->upload(
                        texBox, mVideoTexture[eye], 0, 0, 0, false );

                mVideoTexture[eye]->notifyDataIsReady();
            }
        }
        else
        {
//            LOG << "wait" << LOGEND;
        }

    }

    void GraphicsSystem::setupDebugScreen()
    {
        mDebugCamera = mVRSceneManager->createCamera("Debug Camera");
        mDebugCamera->setAutoAspectRatio(true);
        mDebugCamera->setNearClipDistance(0.01);
        mDebugCamera->setFarClipDistance(1000.0);
        //And attach it to a SceneNode
        mDebugCameraNode = mVRSceneManager->getRootSceneNode()->createChildSceneNode();
        mDebugCamera->detachFromParent();
        mDebugCameraNode->attachObject(mDebugCamera);
        mDebugCamera->setPosition(10,10, 10);
        mDebugCamera->lookAt( 0, 0, 0);
//        mDebugCamera->setDirection(Ogre::Vector3(0,0,0)-mDebugCamera->getPosition());

        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        mDebugWS = compositorManager->addWorkspace(
                mVRSceneManager,
                mDebugWindow->getTexture(),
                mDebugCamera,
                "DebugWorksapce",
                true);
    }

    bool GraphicsSystem::isRenderWindowVisible()
    {
        bool isVisible = false;
        isVisible = isVisible ||
                (mWindow && mWindow->isVisible());
        isVisible = isVisible ||
                (mDebugWindow && mDebugWindow->isVisible());
        return isVisible;
    }

    bool GraphicsSystem::getQuit()
    {
        return mQuit;
    }

    void GraphicsSystem::update( Ogre::uint64 microSecsSinceLast)
    {
        mFrameCnt++;

        const Ogre::uint64 second = 1000000;
        if (microSecsSinceLast > second)
        {
            uploadVideoData2GPU();
        }

        mGameState->update(microSecsSinceLast);

        //TODO: manage Window Messages
        Ogre::WindowEventUtilities::messagePump();
        if(isRenderWindowVisible())
            mQuit |= !mRoot->renderOneFrame();

        // when to uploadVideoData
        // as early as possible so we never effect the framerate
        //alt 2: when we get it
        //alt 3: right after GameState hase updated, so late
        uploadVideoData2GPU();
    }
}
