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

#include "opencv2/opencv.hpp"

#include <experimental/filesystem>
#include <RenderSystems/GL3Plus/OgreGL3PlusPlugin.h>

#include <memory>

namespace esvr2
{
    class OpenVRCompositorListener;

    class VideoLoader;

    class GraphicsSystem
    {
    friend Esvr2;
    friend GameState;
    friend OpenVRCompositorListener;
    public:
        GraphicsSystem( Esvr2 *esvr2, std::shared_ptr<GameState> gameState );
        ~GraphicsSystem();

        // we are overwriting initialize
        void initialize( const Ogre::String &windowTitle );
        void deinitialize(void);

        bool getShowVideo( void ) { return mShowVideo; };
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
        void setDistortion(Distortion dist);

        Ogre::Real getZoom();
        void setZoom( Ogre::Real );

        void beginFrameParallel(void);
        void finishFrameParallel(void) {};
    public:
        Ogre::Window *getRenderWindow();

        void setQuit();
        bool getQuit();

    private:
        void setupResources(void);
        void loadResources(void);
        void registerHlms(void);
        void loadTextureCache(void);
        void loadHlmsDiskCache(void);
        void chooseSceneManager(void);
        void update(float timesincelast);
    private:
        Esvr2 *mEsvr2;
        std::shared_ptr<GameState> mGameState;

        Ogre::SceneManager          *mLaparoscopeSceneManager;
        Ogre::SceneManager          *mVRSceneManager;
        Ogre::Root                  *mRoot;
        // TODO: do we need two windows
        Ogre::Window                *mRenderWindow;
        Ogre::String                mPluginsFolder;
        Ogre::String                mWriteAccessFolder;
        Ogre::String                mResourcePath;

        Ogre::GL3PlusPlugin         *mGL3PlusPlugin;

        Ogre::v1::OverlaySystem     *mOverlaySystem;

        Ogre::SceneNode             *mLaparoscopeCamerasNode;
        Ogre::SceneNode             *mLaparoscopeCameraNode[2];
        Ogre::SceneNode             *mVRCamerasNode;
        Ogre::SceneNode             *mVRCameraNode[2];
        //two real cameras and two workspaces (two cameras rendering) or
        //only use one VR Camera and workspace (Instanced Rendering)
        Ogre::Camera                *mLaparoscopeCameras[2];
        Ogre::Camera                *mVRCameras[2];
        Ogre::Real                  mZoom;
        Ogre::Real                  mCamNear;
        Ogre::Real                  mCamFar;
        Ogre::CompositorWorkspace   *mLaparoscopeWorkspaces[2];
        Ogre::CompositorWorkspace   *mVRWorkspaces[2];
        Ogre::CompositorWorkspace   *mMirrorWorkspace;
        Ogre::Camera                *mVRCullCamera;
        Ogre::TextureGpu            *mVRTexture;
        Ogre::TextureGpu            *mVideoTexture[2];
        Ogre::TextureGpu            *mLaparoscopeViewTexture[2];
        Ogre::VrData                mVrData;
        HmdConfig                   mHmdConfig;
        StereoCameraConfig          mCameraConfig;

        OpenVRCompositorListener    *mOvrCompositorListener;

        // these are nullptr if there is no SteamVR available through OpenVR
        vr::IVRSystem *mHMD;
        vr::IVRCompositor *mVRCompositor;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

        Ogre::Vector4 mVrOffsetScalesRaw[2], mVrOffsetScalesUndistRect[2];
        Ogre::StagingTexture *mStagingTexture[2];

        int mScreen;
        bool mIsStereo;
        size_t mEyeNum;
        bool mShowVideo;

        bool mQuit;

        int mLastFrameUpdate;
        int mUpdateFrames;

        bool mUseMicrocodeCache;
        bool mUseHlmsDiskCache;

        //------------------------------------
        // function
        //------------------------------------
        void createLaparoscopeCameras(void);
        void createVRCameras(void);
        void setupVRCompositor(void);
        void setupLaparoscopeCompositors(void);

        std::string GetTrackedDeviceString(
            vr::TrackedDeviceIndex_t unDevice,
            vr::TrackedDeviceProperty prop,
            vr::TrackedPropertyError *peError = nullptr);
        void initCompositorVR(void);
        void initOpenVR(void);
        void createTwoWorkspaces();
        void setupImageData();

        void syncVRCameraProjection( bool bForceUpdate );

        Ogre::Vector4 getVpOffset( Distortion dist, size_t eye);

        bool calcAlign(void);
    };
}

#endif
