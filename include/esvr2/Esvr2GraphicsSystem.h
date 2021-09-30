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

    //! Class managing the Ogre Systems as well as the Gamestate
    /*!
     * \addtogroup Components
     */
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
        //! \brief parse the Ogre resource config file
        //! mostly copied from Ogre-Examples
        void setupResources(void);
        //! \brief load Ogre resources
        //! mostly copied from Ogre-Examples
        void loadResources(void);
        //! \brief register the Hlms Systems
        //! mostly copied from Ogre-Examples
        void registerHlms(void);
        //! mostly copied from Ogre-Examples
        void loadTextureCache(void);
        //! mostly copied from Ogre-Examples
        void loadHlmsDiskCache(void);
        //! \brief creates the SceneManagere Instances for the VR and the Laparoscopic Environments
        void createSceneManager(void);
        //! \brief create the virtual cameras for the overlays in the images
        void createLaparoscopeCameras(void);
        //! \brief create the virtual cameras for the VR-Environment
        void createVRCameras(void);
        //! \brief setup the data structures where the laparoscopic images go to
        void setupImageData(void);
        //! \brief when Debugging create the Camerea, CompositorManager and the SceneManager
        void setupDebugScreen();

        //! \brief Upload Videodata to GPU
        void uploadVideoData2GPU(void);
        //! \brief setup the VR-Textures (memory in the GPU to render VR-Environment to)
        void setupVRTextures(void);
        //! \brief setup the CompositorManagers
        void setupVRCompositor(void);
        //! \brief setup the Laparoscope-Textures (memory in the GPU to copy Laparoscope Image to)
        void setupLaparoscopeTextures();
        //! \brief setup the CompositorManagers for the overlays onto Laparoscope images
        void setupLaparoscopeCompositors(void);
        //! \brief setup the Menuplane-Textures (memory in the GPU to copy Laparoscope Image to)
        void setupInfoScreenTextures();
        //! \brief setup the CompositorManagers for the overlays in Laparoscope
        void setupInfoScreenCompositor();

        //! \brief callback for SDL-WindowEvents
        void handleWindowEvent( const SDL_Event& evt );
        //! \brief callback for SDL-Events
        /*! handles quit and WindowEvent and pass them on to GameState
         */
        void pumpSDLEvents();
#ifdef USE_FOOTPEDAL
        void pumpFootPedalEvents();
#endif // USE_FOOTPEDAL


    public:
        GraphicsSystem( Esvr2 *esvr2);
        ~GraphicsSystem() {};

        bool initialize( ) override;
        void deinitialize(void) override;

        void update(uint64 startTimeMs);

        //Project getters and setters
        GameState *getGameState();

        //Ogre getters and setters
        //! \brief quit the application and write out a settings Log
        void quit() override;
        //! \brief checks using SDL Window if it is visible
        bool isRenderWindowVisible( void );
        //! \brief checks if the Laparoscope-videostream is running
        bool getShowVideo( void ) { return mShowVideo; };
        //! \brief toggle playing Laparoscope-videostream
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
    };
}

#endif
