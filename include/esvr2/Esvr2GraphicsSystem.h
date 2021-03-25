#ifndef _Esvr2_GraphicsSystem_H_
#define _Esvr2_GraphicsSystem_H_

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2PoseState.h"
#include "Esvr2InteractiveElement2DDef.h"
#include "Esvr2FootPedal.h"

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

    class GraphicsSystem :
            public virtual Component
    {
    friend GameState;
    friend OpenVRCompositorListener;
    private:
        Esvr2 *mEsvr2;

        //Esvr2 Components
        GameState *mGameState {nullptr};
        OpenVRCompositorListener    *mOvrCompositorListener {nullptr};

        //Ogre
        Ogre::Root                  *mRoot {nullptr};
        Ogre::GL3PlusPlugin         *mGL3PlusPlugin {nullptr};
        Ogre::Window                *mWindow {nullptr};
        Ogre::String                mWindowTitle {"Esvr2"};

        //Other Ogre stuff
        Ogre::String                mPluginsFolder {""};
        Ogre::String                mWriteAccessFolder {""};
        bool                        mUseMicrocodeCache {true};
        bool                        mUseHlmsDiskCache {true};
        bool                        mLoadCacheSucc {false};

        //Ogre Workspaces
        Ogre::CompositorWorkspace *mLaparoscopeWorkspaces[2] {nullptr, nullptr};
        Ogre::CompositorWorkspace *mVRWorkspaces[2] {nullptr, nullptr};
        Ogre::CompositorWorkspace   *mMirrorWorkspace {nullptr};
        Ogre::CompositorWorkspace   *mInfoScreenWorkspace {nullptr};

        //Ogre SceneManger
        Ogre::SceneManager          *mLaparoscopeSceneManager {nullptr};
        Ogre::SceneManager          *mVRSceneManager {nullptr};
        Ogre::SceneManager          *mEmptySceneManager {nullptr};

        Ogre::v1::OverlaySystem     *mOverlaySystem {nullptr};

        //Ogre Cameras
        //two real cameras and two workspaces (two cameras rendering) or
        //only use one VR Camera and workspace (Instanced Rendering)
        Ogre::Camera                *mLaparoscopeCameras[2] {nullptr, nullptr};
        Ogre::Real                  mVRCameraNear {0.3}, mVRCameraFar {30.0};
        Ogre::Camera                *mVRCameras[2] {nullptr, nullptr};
        Ogre::Camera                *mVRCullCamera {nullptr};
        Ogre::VrData                mVrData;

        //Ogre Textures
        Ogre::TextureGpu            *mVRTexture {nullptr};
        Ogre::TextureGpu            *mVideoTexture[2] {nullptr, nullptr};
        Ogre::TextureGpu            *mLaparoscopeViewTexture[2] {nullptr, nullptr};
        Ogre::TextureGpu            *mInfoScreenTexture {nullptr};
        Ogre::StagingTexture        *mStagingTextures[2] {nullptr, nullptr};

        //Ogre Debug stuff
        Ogre::Window                *mDebugWindow {nullptr};
        Ogre::CompositorWorkspace   *mDebugWS {nullptr};
        Ogre::Camera                *mDebugCamera {nullptr};
        Ogre::SceneNode             *mDebugCameraNode {nullptr};

        //SDL for input and output
        SDL_Window                  *mSdlWindow {nullptr};
#ifdef USE_FOOTPEDAL
        FootPedal *mFootPedal {nullptr};
#endif //USE_FOOTPEDAL

        //Class Variables
        InteractiveElementConfig mInteractiveElementConfig;
        bool mQuit {false};
        size_t mEyeNum;
        bool mShowVideo {true};

        int mFrameCnt {0};
        int mLastFrameUpdate {0};
        int mVideoUpdateFrames {0};

        Ogre::uint64 mLastStartTime {0};

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
#ifdef USE_FOOTPEDAL
        void pumpFootPedalEvents();
#endif // USE_FOOTPEDAL

        void setupInfoScreenTextures();
        void setupInfoScreenCompositor();

    public:
        GraphicsSystem( Esvr2 *esvr2);
        ~GraphicsSystem() {};

        bool initialize( ) override;
        void deinitialize(void) override;

        void update(uint64 startTimeMs);

        //Project getters and setters
        GameState *getGameState();

        //Ogre getters and setters

        //get variables
        void quit() override;
        bool getQuit() override;
        bool isRenderWindowVisible( void );
        bool getShowVideo( void ) { return mShowVideo; };
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
    };
}

#endif
