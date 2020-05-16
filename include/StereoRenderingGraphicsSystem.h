
#ifndef _Demo_StereoRenderingGraphicsSystem_H_
#define _Demo_StereoRenderingGraphicsSystem_H_

#include "StereoRendering.h"
#include "StereoRenderingVideoLoader.h"

#include "GraphicsSystem.h"
#include "OgreSceneNode.h"
#include "OgreCamera.h"
#include "Compositor/OgreCompositorManager2.h"

#include "openvr.h"

#include "OgreStagingTexture.h"

#include "OgreMatrix4.h"

#include "opencv2/opencv.hpp"

#include <experimental/filesystem>
#include <mutex>


namespace Demo
{
    class OpenVRCompositorListener;

    class VideoLoader;

    class StereoGraphicsSystem : public GraphicsSystem
    {
    private:
        //Depending on this type start with different Compositor setup
        WorkspaceType               mWorkSpaceType;

        Ogre::SceneNode             *mCamerasNode;
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
        Ogre::VrData                mVrData;
        HmdConfig                   mHmdConfig;

        OpenVRCompositorListener    *mOvrCompositorListener;

        // these are nullptr if there is no SteamVR available through OpenVR
        vr::IVRSystem *mHMD;
        vr::IVRCompositor *mVRCompositor;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

        VideoLoader *mVideoSource;
        VideoRenderTarget mVideoTarget;
        size_t mCameraWidth[2], mCameraHeight[2];
        //left_left, left_right right_left right_right
        //left_top left_bottom  right_top right_bottom
        struct ImageRenderConfig {
            int leftAlign[2];
            int topAlign[2];
            cv::Size size[2];
        } *mImageRenderConfig;
        std::mutex mMtxImageResize;
        cv::Mat mImageResize[2];
        Ogre::uint8 *mImageData[2];
        size_t mImageDataSize[2];
        Ogre::StagingTexture *mStagingTexture[2];

        //used for mDrawHelpers
        bool mDrawHelpers;
        int mCVr[2][2];
        int mImgMiddleResize[2][2];

        bool mIsStereo;
        size_t mEyeNum;
        bool mShowVideo;

        int mLastFrameUpdate;
        int mUpdateFrames;
        //------------------------------------
        // function
        //------------------------------------
        void createCamera(void);
        Ogre::CompositorWorkspace* setupCompositor(void);

        std::string GetTrackedDeviceString(
            vr::TrackedDeviceIndex_t unDevice,
            vr::TrackedDeviceProperty prop,
            vr::TrackedPropertyError *peError = nullptr);
        void initCompositorVR(void);
        void initOpenVR(void);
        void setupImageData();

        void syncCameraProjection( bool bForceUpdate );

        bool fillTexture(void);
        bool clearTexture(void);


    public:
        StereoGraphicsSystem(
            GameState *gameState,
            WorkspaceType wsType,
            HmdConfig hmdConfig,
            bool mIsStereo,
            bool showOgreDialog = false,
            bool showVideo = true,
            Demo::VideoRenderTarget renderVideoTarget = TO_SQUARE,
            Ogre::Real camNear = 0.1f, Ogre::Real camFar = 200.0f);
        virtual void deinitialize(void);

        void setImgPtr(const cv::Mat *left, const cv::Mat *right);
        bool calcAlign(CameraConfig &mCameraConfig);

        virtual void beginFrameParallel(void);

        void _notifyVideoSource( VideoLoader *videoSource )
        {
            mVideoSource = videoSource;
        };
    };
}

#endif
