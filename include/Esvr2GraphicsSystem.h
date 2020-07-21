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
#include <mutex>

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

        OpenVRCompositorListener    *mOvrCompositorListener;

        // these are nullptr if there is no SteamVR available through OpenVR
        vr::IVRSystem *mHMD;
        vr::IVRCompositor *mVRCompositor;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
        PoseState *mCameraPoseState;

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

        cv::Mat mUndistortMap1[2], mUndistortMap2[2];
        cv::Mat mUndistortRectifyMap1[2], mUndistortRectifyMap2[2];

        //used for mDrawHelpers
        bool mDrawHelpers;
        int mCVr[2][2];
        int mImgMiddleResize[2][2];

        int mScreen;
        bool mIsStereo;
        size_t mEyeNum;
        bool mShowVideo;

        Distortion mInputDistortion;
        Distortion mOutputDistortion;

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


    public:
        GraphicsSystem(
            Demo::GameState *gameState,
            WorkspaceType wsType,
            Ogre::VrData *vrData,
            HmdConfig hmdConfig,
            int screen,
            Distortion inputDistortion,
            bool mIsStereo,
            bool showOgreDialog = false,
            bool showVideo = true,
            esvr2::VideoRenderTarget renderVideoTarget = VRT_TO_SQUARE,
            Ogre::Real camNear = 0.005f, Ogre::Real camFar = 200.0f);
        virtual void deinitialize(void);

        // we are overwriting initialize
        void initialize( const Ogre::String &windowTitle );

        void setImgPtr(const cv::Mat *left, const cv::Mat *right);
        bool getShowVideo( void ) { return mShowVideo; };
        void toggleShowVideo( void ) { mShowVideo = !mShowVideo; };
        void itterateDistortion( void );
        Distortion getDistortion( void ) {return mOutputDistortion;};
        bool calcAlign(StereoCameraConfig &mCameraConfig);

        virtual void beginFrameParallel(void);

        void _notifyVideoSource( VideoLoader *videoSource )
        {
            mVideoSource = videoSource;
        };

        void _notifyPoseSource( PoseState *poseState )
        {
            mCameraPoseState = poseState;
        };
    };
}

#endif