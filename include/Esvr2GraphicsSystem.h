#ifndef _Esvr2_GraphicsSystem_H_
#define _Esvr2_GraphicsSystem_H_

#include "Esvr2StereoRendering.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2PoseState.h"

#include "GraphicsSystem.h"

#include "OgreSceneNode.h"
#include "OgreCamera.h"
#include "Compositor/OgreCompositorManager2.h"

#include "openvr.h"

#include "OgreStagingTexture.h"

#include "OgreMatrix4.h"

#include "opencv2/opencv.hpp"

#include <experimental/filesystem>

namespace esvr2
{
    class OpenVRCompositorListener;

    class VideoLoader;

    class GraphicsSystem : public Demo::GraphicsSystem
    {
    private:
        //Depending on this type start with different Compositor setup
        WorkspaceType               mWorkSpaceType;

        Ogre::SceneNode             *mCamerasNodeTrans;
        Ogre::SceneNode             *mCamerasNode;
        Ogre::SceneNode             *mCameraNode[2];
        //two real cameras and two workspaces (two cameras rendering) or
        //only use one VR Camera and workspace (Instanced Rendering)
        Ogre::Camera                *mEyeCameras[2];
        Ogre::Real                  mCamNear;
        Ogre::Real                  mCamFar;
        Ogre::CompositorWorkspace   *mVrWorkspaces[2];
        Ogre::CompositorWorkspace   *mMirrorWorkspace;
        Ogre::Camera                *mVrCullCamera;
        Ogre::TextureGpu            *mVrTexture;
        Ogre::TextureGpu            *mVideoTexture[2];
        Ogre::VrData                *mVrData;
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
        PoseState *mCameraPoseState;

        VideoLoader *mVideoLoader;
        VideoRenderTarget mVideoTarget;
        size_t mCameraWidth[2], mCameraHeight[2];
        //left_left, left_right right_left right_right
        //left_top left_bottom  right_top right_bottom
        struct ImageRenderConfig {
            int leftAlign[2];
            int topAlign[2];
            cv::Size size[2];
        } *mImageRenderConfig;
        Ogre::StagingTexture *mStagingTexture[2];

        //used for mDrawHelpers
        bool mDrawHelpers;
        int mCVr[2][2];
        int mImgMiddleResize[2][2];

        int mScreen;
        bool mIsStereo;
        size_t mEyeNum;
        bool mShowVideo;

        int mLastFrameUpdate;
        int mUpdateFrames;
        //------------------------------------
        // function
        //------------------------------------
        void createCamera(void);
        void alignCameras(void);
        Ogre::CompositorWorkspace* setupCompositor(void);

        std::string GetTrackedDeviceString(
            vr::TrackedDeviceIndex_t unDevice,
            vr::TrackedDeviceProperty prop,
            vr::TrackedPropertyError *peError = nullptr);
        void initCompositorVR(void);
        void initOpenVR(void);
        void createTwoWorkspaces();
        void setupImageData();

        void syncCameraProjection( bool bForceUpdate );

        bool fillTexture(void);
        bool clearTexture(void);

        bool calcAlign(void);

    public:
        GraphicsSystem(
            Demo::GameState *gameState,
            WorkspaceType wsType,
            Ogre::VrData *vrData,
            HmdConfig hmdConfig,
            VideoLoader *videoLoader,
            int screen,
            bool isStereo,
            bool showOgreDialog = false,
            bool showVideo = true,
            esvr2::VideoRenderTarget renderVideoTarget = VRT_TO_SQUARE,
            Ogre::Real camNear = 0.005f, Ogre::Real camFar = 200.0f);
        virtual void deinitialize(void);

        // we are overwriting initialize
        void initialize( const Ogre::String &windowTitle );

        bool getShowVideo( void ) { return mShowVideo; };
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
        void itterateDistortion( void );

        VideoLoader *getVideoLoader() { return mVideoLoader; };

        virtual void beginFrameParallel(void);

        void _notifyPoseSource( PoseState *poseState )
        {
            mCameraPoseState = poseState;
        };
    };
}

#endif
