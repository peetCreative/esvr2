#ifndef _Esvr2_GraphicsSystem_H_
#define _Esvr2_GraphicsSystem_H_

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2PoseState.h"

#include "Ogre.h"
#include "OgreSceneNode.h"
#include "OgreCamera.h"
#include "Compositor/OgreCompositorManager2.h"
#include "RenderSystems/GL3Plus/OgreGL3PlusPlugin.h"
#include <Overlay/OgreOverlaySystem.h>

#include "openvr.h"

#include "OgreStagingTexture.h"

#include "OgreMatrix4.h"

#include <SDL.h>

#include "opencv2/opencv.hpp"

#include <experimental/filesystem>
#include <RenderSystems/GL3Plus/OgreGL3PlusPlugin.h>


namespace esvr2
{
    class OpenVRCompositorListener;

    class VideoLoader;

    class GraphicsSystem
    {
    friend GameState;
    friend OpenVRCompositorListener;
    private:
        Esvr2 *mEsvr2;

        //Esvr2 Components
        GameState *mGameState;
        OpenVRCompositorListener    *mOvrCompositorListener;

        //Ogre
        Ogre::Root                  *mRoot;
        Ogre::GL3PlusPlugin         *mGL3PlusPlugin;
        Ogre::Window                *mWindow;
        Ogre::String                mWindowTitle;

        //Other Ogre stuff
        Ogre::String                mPluginsFolder;
        Ogre::String                mWriteAccessFolder;
        bool mUseMicrocodeCache;
        bool mUseHlmsDiskCache;

        //Ogre Workspaces
        Ogre::CompositorWorkspace   *mLaparoscopeWorkspaces[2];
        Ogre::CompositorWorkspace   *mVRWorkspaces[2];
        Ogre::CompositorWorkspace   *mMirrorWorkspace;
        Ogre::CompositorWorkspace   *mInfoScreenWorkspace;

        //Ogre SceneManger
        Ogre::SceneManager          *mLaparoscopeSceneManager;
        Ogre::SceneManager          *mVRSceneManager;
        Ogre::SceneManager          *mEmptySceneManager;

        Ogre::v1::OverlaySystem     *mOverlaySystem;

        //Ogre Cameras
        //two real cameras and two workspaces (two cameras rendering) or
        //only use one VR Camera and workspace (Instanced Rendering)
        Ogre::Camera                *mLaparoscopeCameras[2];
        Ogre::Real                  mVRCameraNear, mVRCameraFar;
        Ogre::Camera                *mVRCameras[2];
        Ogre::Camera                *mVRCullCamera;
        Ogre::VrData                mVrData;

        //Ogre Textures
        Ogre::TextureGpu            *mVRTexture;
        Ogre::TextureGpu            *mVideoTexture[2];
        Ogre::TextureGpu            *mLaparoscopeViewTexture[2];
        Ogre::TextureGpu            *mInfoScreenTexture;
        Ogre::StagingTexture        *mStagingTextures[2];

        //Ogre Debug stuff
        Ogre::Window *mDebugWindow;
        Ogre::CompositorWorkspace *mDebugWS;
        Ogre::Camera *mDebugCamera;
        Ogre::SceneNode *mDebugCameraNode;

        //SDL for input and output
        SDL_Window *mSdlWindow;

        //Class Variables
        bool mQuit;
        size_t mEyeNum;
        bool mShowVideo;

        int mFrameCnt;
        int mLastFrameUpdate;
        int mVideoUpdateFrames;

        //intern functions
        void setupResources(void);
        void loadResources(void);
        void registerHlms(void);
        void loadTextureCache(void);
        void loadHlmsDiskCache(void);
        void chooseSceneManager(void);
        void createLaparoscopeCameras(void);
        void createVRCameras(void);
        void setupImageData(void);
        void setupDebugScreen();

        void uploadVideoData2GPU(void);
        void setupVRTextures(void);
        void setupVRCompositor(void);
        void setupLaparoscopeTextures();
        void setupLaparoscopeCompositors(void);

        void handleWindowEvent( const SDL_Event& evt );
        void pumpSDLEvents();

        bool configureLaparoscopeCamera(void);
        bool configureVRCamera(void);

        void setupInfoScreenTextures();
        void setupInfoScreenCompositor();

    public:
        GraphicsSystem( Esvr2 *esvr2);
        ~GraphicsSystem() {};

        void initialize( );
        void deinitialize(void);

        void update(Ogre::uint64 microSecsSinceLast);

        //Project getters and setters
        GameState *getGameState();

        //Ogre getters and setters

        //get variables
        bool getQuit();
        bool isRenderWindowVisible( void );
        bool getShowVideo( void ) { return mShowVideo; };
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
        void setDistortion(Distortion dist);

        Ogre::Real getZoom();
        void setZoom( Ogre::Real );
    };
}

#endif
