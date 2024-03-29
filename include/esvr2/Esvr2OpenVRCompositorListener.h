#ifndef _Esvr2_OpenVRCompositorListener_H_
#define _Esvr2_OpenVRCompositorListener_H_

#include "Esvr2.h"
#include "Esvr2PoseState.h"
#include "Esvr2GraphicsSystem.h"

#include "OgreCamera.h"
#include "OgreFrameListener.h"
#include "Compositor/OgreCompositorWorkspaceListener.h"

#if __cplusplus <= 199711L
    #ifndef nullptr
        #define OgreDemoNullptrDefined
        #define nullptr (0)
    #endif
#endif
#include "openvr.h"
#if __cplusplus <= 199711L
    #ifdef OgreDemoNullptrDefined
        #undef OgreDemoNullptrDefined
        #undef nullptr
    #endif
#endif

namespace esvr2
{
    //! \brief mode defining, when to update the tracking position in the renderpipeline
    //! for more information about this see OpenVR example in Ogre 2.2
    enum VrWaitingMode
    {
        AfterSwap,
        BeforeSceneGraph,
        AfterSceneGraph,
        BeforeShadowmaps,
        BeforeFrustumCulling,
        AfterFrustumCulling,
        NumVrWaitingModes
    };

    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix34_t matPose );
    Ogre::Matrix4 convertSteamVRMatrixToMatrix( vr::HmdMatrix44_t matPose );

    class OpenVRCompositorListener : public Ogre::FrameListener, public Ogre::CompositorWorkspaceListener
    {
    friend GraphicsSystem;
    private:
        GraphicsSystem          *mGraphicsSystem;
        Ogre::RenderSystem      *mRenderSystem;

        // these are nullptr if there is no SteamVR available through OpenVR
        vr::IVRSystem *mHMD;
        vr::IVRCompositor *mVRCompositor;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

        Ogre::Matrix4           mDevicePose[vr::k_unMaxTrackedDeviceCount];
        vr::ETextureType        mApiTextureType;
        Ogre::Vector3           mCullCameraOffset;

        int mValidPoseCount;

        VrWaitingMode mWaitingMode;
        VrWaitingMode mFirstGlitchFreeMode;
        bool                    mMustSyncAtEndOfFrame;

        bool mTrackPose;
        bool mWriteTexture;

        void syncCamera(void);
        void syncVRCameraProjection( bool bForceUpdate );
        void updateHmdTrackingPose(void);
//      TODO: use these functions, when we go to moving camera
//         void syncCullCamera(void);
        std::string GetTrackedDeviceString(
                vr::TrackedDeviceIndex_t unDevice,
                vr::TrackedDeviceProperty prop,
                vr::TrackedPropertyError *peError = nullptr);

    public:
        OpenVRCompositorListener(
                GraphicsSystem *graphicsSystem );
        virtual ~OpenVRCompositorListener();

        bool initOpenVR(void);

        virtual bool frameStarted( const Ogre::FrameEvent& evt );
        virtual bool frameRenderingQueued( const Ogre::FrameEvent &evt );
        virtual bool frameEnded( const Ogre::FrameEvent& evt );

        virtual void workspacePreUpdate( Ogre::CompositorWorkspace *workspace );
        virtual void passPreExecute( Ogre::CompositorPass *pass );
        virtual void passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass );
        virtual void passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass );

        /// See VrWaitingMode::VrWaitingMode
        void setWaitingMode( VrWaitingMode waitingMode );
        VrWaitingMode getWaitingMode(void)   { return mWaitingMode; }

        void triggerWriteTexture(){mWriteTexture = true;};

        /** Remarks from Ogre VR-Example:
         * When operating in VrWaitingMode::AfterSceneGraph or later, there's a chance
            graphical artifacts appear if the camera transform is immediately changed after
            calling WaitGetPoses instead of waiting for the next frame.

            This is a question of whether you want to prioritize latency over graphical
            artifacts, or an artifact-free rendering in exchange for up to one frame of
            latency. The severity and frequency of artifacts may vastly depend on the game.

            Because the graphical artifacts get more severe with each mode, you may e.g.
            find acceptable to tolerate artifacts in VrWaitingMode::BeforeFrustumCulling,
            but not in VrWaitingMode::AfterFrustumCulling
        @param glitchFree
            First stage to consider glitch-free. e.g. calling:
                setGlitchFree( VrWaitingMode::AfterSwap );
            Means that all waiting modes must behave glitch-free, while calling:
                setGlitchFree( VrWaitingMode::BeforeSceneGraph );
            Means that only AfterSwap is allowed to have glitches, while:
                setGlitchFree( VrWaitingMode::NumVrWaitingModes );
            Means that any waiting mode is allowed to have glitches
        */
        void setGlitchFree( VrWaitingMode firstGlitchFreeMode );

        /** Remarks from Ogre VR-Example:
         * Returns true if the current waiting mode can update the camera immediately,
            false if it must wait until the end of the frame.
        @remarks
            VrWaitingMode::BeforeSceneGraph and earlier always returns true.

            See OpenVRCompositorListener::setGlitchFree
        */
        bool canSyncCameraTransformImmediately(void) const;

    };
}

#endif
