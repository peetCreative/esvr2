#include "Esvr2GraphicsSystem.h"

#include "Esvr2.h"
#include "Esvr2GameState.h"
#include "Esvr2OpenVRCompositorListener.h"
#include "Esvr2ParseYml.h"
#include "Esvr2FootPedal.h"
#include "Esvr2Helper.h"

#include "Ogre.h"
#include "OgrePlatform.h"
#include "OgreTextureGpuManager.h"
#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
#include "OgreResource.h"
#include "OgrePlatformInformation.h"
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

#include <SDL.h>
#include <SDL_syswm.h>

#include <sstream>
#include <mutex>
#include <vector>

namespace esvr2
{
    GraphicsSystem::GraphicsSystem(
            Esvr2 *esvr2):
            mEsvr2(esvr2),
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 )
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

            Ogre::ConfigFile::SettingsMultiMap::iterator i;
            for (i = settings->begin(); i != settings->end(); ++i)
            {
                typeName = i->first;
                archName = i->second;
                if (secName == "Custom")
                {
                    readInteractiveElementConfigYml(
                            archName, mInteractiveElementConfig);
                }
                else if( secName != "Hlms" )
                {
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

        Ogre::HlmsUnlit *hlmsUnlit = nullptr;
        Ogre::HlmsPbs *hlmsPbs = nullptr;

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
    void GraphicsSystem::createSceneManager(void)
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
        // Even though this scene will be empty we need this
        mEmptySceneManager = mRoot->createSceneManager( Ogre::ST_GENERIC,
                                                     1,
                                                     "Empty" );


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
                RealArray16 *vec = hmdConfig.projectionMatrixPtr[eye];
                Ogre::Matrix4 projMat = RealArray16ToMatrix4(*vec);
                mVRCameras[eye]->setCustomProjectionMatrix(true, projMat);
                RealArray3 *vec1 = hmdConfig.eyeToHeadPtr[eye];
                Ogre::Vector3 pose = RealArray3ToVector3(*vec1);
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
    bool GraphicsSystem::initialize()
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
        if( SDL_Init( SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_EVENTS ) != 0 )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR, "Cannot initialize SDL2!",
                         "GraphicsSystem::initialize" );
        }

#ifdef USE_FOOTPEDAL
        mFootPedal = new FootPedal(mEsvr2->mConfig->serialPort);
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
                return false;
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

        //needed fro sdl2
        int screen = 0;
        int posX = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);
        int posY = SDL_WINDOWPOS_CENTERED_DISPLAY(screen);

        if(fullscreen)
        {
            posX = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
            posY = SDL_WINDOWPOS_UNDEFINED_DISPLAY(screen);
        }

        mSdlWindow = SDL_CreateWindow(
                mWindowTitle.c_str(),    // window title
                posX,               // initial x position
                posY,               // initial y position
                width,              // width, in pixels
                height,             // height, in pixels
                SDL_WINDOW_SHOWN
                | (fullscreen ? SDL_WINDOW_FULLSCREEN : 0) | SDL_WINDOW_RESIZABLE );

        //Get the native whnd
        SDL_SysWMinfo wmInfo;
        SDL_VERSION( &wmInfo.version );

        if( SDL_GetWindowWMInfo( mSdlWindow, &wmInfo ) == SDL_FALSE )
        {
            OGRE_EXCEPT( Ogre::Exception::ERR_INTERNAL_ERROR,
                         "Couldn't get WM Info! (SDL2)",
                         "GraphicsSystem::initialize" );
        }
        Ogre::String winHandle =
                Ogre::StringConverter::toString( (uintptr_t)wmInfo.info.x11.window );
        params.insert( std::make_pair(
                "SDL2x11", Ogre::StringConverter::toString( (uintptr_t)&wmInfo.info.x11 ) ) );
        params.insert( std::make_pair("parentWindowHandle",  winHandle) );

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

        //we need to do this before setup resources,
        //for no reason
        createSceneManager();
        mOverlaySystem = OGRE_NEW Ogre::v1::OverlaySystem();
        mEmptySceneManager->addRenderQueueListener( mOverlaySystem );
        mEmptySceneManager->getRenderQueue()->setSortRenderQueue(
                Ogre::v1::OverlayManager::getSingleton().mDefaultRenderQueueId,
                Ogre::RenderQueue::StableSort );


        //Resources
        setupResources();
        loadResources();
        mLoadCacheSucc = readCacheYml(mEsvr2->mConfig);

        //we need to create them here because SceneManager needs them
        setupLaparoscopeTextures();
        setupInfoScreenTextures();

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
        createVRCameras();
        createLaparoscopeCameras();
        //create Gamestate
        mGameState = new GameState(mEsvr2, this);
        mGameState->loadDatablocks();
        mGameState->createLaparoscopeScene();
        mGameState->createVRScene();
        mGameState->addSettingsEventLog("init");

        if (mEsvr2->mConfig->debugScreen)
        {
            setupDebugScreen();
        }
        else
        {
            setupVRCompositor();
        }
        setupInfoScreenCompositor();
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
        mReady = true;
        return true;
    }

    void GraphicsSystem::deinitialize(void)
    {

        delete mOvrCompositorListener;
        mOvrCompositorListener = nullptr;

        Ogre::TextureGpuManager *textureManager =
                mRoot->getRenderSystem()->getTextureGpuManager();

        if( mVRTexture )
        {
            textureManager->destroyTexture( mVRTexture );
            mVRTexture = nullptr;
        }

        for( size_t eye = 0; eye < 2; eye++ )
        {
            if( mStagingTextures[eye] )
            {
                //Tell the TextureGpuManager we're done with this StagingTexture. Otherwise it will leak.
                textureManager->removeStagingTexture(mStagingTextures[eye] );
                mStagingTextures[eye] = nullptr;
            }

            //Don't need to be destroyed compositorManager is doing it
            mVideoTexture[eye] = nullptr;

            if( mLaparoscopeCameras[eye] )
            {
                mLaparoscopeSceneManager->destroyCamera(mLaparoscopeCameras[eye] );
                mLaparoscopeCameras[eye] = nullptr;
            }
        }

        if( mVRCullCamera )
        {
            mVRSceneManager->destroyCamera( mVRCullCamera );
            mVRCullCamera = nullptr;
        }

        Ogre::CompositorManager2 *compositorManager2 =
                mRoot->getCompositorManager2();
        compositorManager2->removeAllWorkspaces();
//        GraphicsSystem::deinitialize();

        if( mSdlWindow )
        {
            // Restore desktop resolution on exit
            SDL_SetWindowFullscreen( mSdlWindow, 0 );
            SDL_DestroyWindow( mSdlWindow );
            mSdlWindow = nullptr;
        }

        SDL_Quit();
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
            mVideoTexture[eye]->setPixelFormat(Ogre::PFG_BGRA8_UNORM);
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
            if (!imgData.img[LEFT].valid() ||
                (mEsvr2->mConfig->isStereo && !imgData.img[RIGHT].valid()))
            {
                LOG << "skip non valid memory" << LOGEND;
                return;
            }

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
                        texBox, mVideoTexture[eye], 0, nullptr, nullptr, false );

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

    void GraphicsSystem::quit()
    {
        LOG << "write Settings Event Log to File " << mEsvr2->mConfig->logFolder << LOGEND;
        writeSettingsEventLog(
                mEsvr2->mConfig->logFolder,
                mEsvr2->mConfig->logPrefix,
                mGameState->mSettingsEventLogs,
                mLastStartTime);
        writeCacheYml(mEsvr2->mConfig);
        mQuit = true;
    }

    void GraphicsSystem::handleWindowEvent( const SDL_Event& evt )
    {
        Ogre::Window *window = mEsvr2->mConfig->debugScreen ?
                mDebugWindow : mWindow;
        switch( evt.window.event )
        {
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                int w,h;
                SDL_GetWindowSize( mSdlWindow, &w, &h );
                window->requestResolution( w, h );
                window->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_RESIZED:
                window->requestResolution( evt.window.data1, evt.window.data2 );
                window->windowMovedOrResized();
                break;
            case SDL_WINDOWEVENT_CLOSE:
                mQuit = true;
                break;
            case SDL_WINDOWEVENT_SHOWN:
                window->_setVisible( true );
                break;
            case SDL_WINDOWEVENT_HIDDEN:
                window->_setVisible( false );
                break;
            case SDL_WINDOWEVENT_ENTER:
                mGameState->mMouseInWindow = true;
                break;
            case SDL_WINDOWEVENT_LEAVE:
                mGameState->mMouseInWindow = false;
                break;
            case SDL_WINDOWEVENT_FOCUS_GAINED:
                mGameState->mWindowHasFocus = true;
                window->setFocused( true );
                break;
            case SDL_WINDOWEVENT_FOCUS_LOST:
                mGameState->mWindowHasFocus = false;
                window->setFocused( false );
                break;
        }
    }

     void GraphicsSystem::pumpSDLEvents()
     {
         SDL_Event evt;
         while( SDL_PollEvent( &evt ) )
         {
             switch( evt.type )
             {
                 case SDL_WINDOWEVENT:
                     handleWindowEvent( evt );
                     break;
                 case SDL_QUIT:
                     mQuit = true;
                     break;
                 default:
                     break;
             }

             mGameState->handleSdlEvent( evt );
         }
     }

#ifdef USE_FOOTPEDAL
    void GraphicsSystem::pumpFootPedalEvents()
    {
        FootPedalEvent evt;
        while( mFootPedal->pollFootPedalEvent( evt ) )
        {
            mGameState->handleFootPedalEvent( evt );
        }
    }
#endif //USE_FOOTPEDAL

    void GraphicsSystem::update( uint64 msStartTime)
    {
        mFrameCnt++;
        Ogre::uint64  msSinceLast = msStartTime - mLastStartTime;
        msSinceLast = msSinceLast < 1 ? msSinceLast : 1;
        mLastStartTime = msStartTime;

        Ogre::WindowEventUtilities::messagePump();
        pumpSDLEvents();
#ifdef USE_FOOTPEDAL
        pumpFootPedalEvents();
#endif //USE_FOOTPEDAL
        const Ogre::uint64 second = 1000000;
        if (msSinceLast > second)
        {
            uploadVideoData2GPU();
        }

        mGameState->update(msSinceLast);

        //TODO: manage Window Messages
        if(isRenderWindowVisible())
            mQuit |= !mRoot->renderOneFrame();

        // when to uploadVideoData
        // as early as possible so we never effect the framerate
        //alt 2: when we get it
        //alt 3: right after GameState hase updated, so late
        uploadVideoData2GPU();
    }

    void GraphicsSystem::setupInfoScreenCompositor()
    {
        Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();

        Ogre::Camera *emptyCamera =
                mEmptySceneManager->createCamera("EmptyCamera");
        mInfoScreenWorkspace = compositorManager->addWorkspace(
                mEmptySceneManager,
                mInfoScreenTexture,
                emptyCamera, "InfoScreenWorkspace",
                true );
    }

    void GraphicsSystem::setupInfoScreenTextures()
    {
        Ogre::TextureGpuManager *textureManager = mRoot->getRenderSystem()->getTextureGpuManager();
        //Radial Density Mask requires the VR texture to be UAV & reinterpretable
        mInfoScreenTexture = textureManager->createOrRetrieveTexture(
                "Info Screen Texture",
                Ogre::GpuPageOutStrategy::Discard,
                Ogre::TextureFlags::RenderToTexture|
                Ogre::TextureFlags::Uav|
                Ogre::TextureFlags::Reinterpretable,
                Ogre::TextureTypes::Type2D );
        mInfoScreenTexture->setResolution( 1080, 720 );
        mInfoScreenTexture->setPixelFormat( Ogre::PFG_RGBA8_UNORM );
        mInfoScreenTexture->scheduleTransitionTo(
                Ogre::GpuResidency::Resident );
    }

    GameState *GraphicsSystem::getGameState()
    {
        return mGameState;
    }

}
