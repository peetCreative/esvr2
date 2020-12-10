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
    namespace VrWaitingMode
    {
        //for more information about this see OpenVR example in Ogre 2.2
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
    }

    class OpenVRCompositorListener : public Ogre::FrameListener, public Ogre::CompositorWorkspaceListener
    {
    protected:
        vr::IVRSystem		*mHMD;
        vr::IVRCompositor	*mVrCompositor;

        Ogre::TextureGpu	*mVrTexture;
        Ogre::Root          *mRoot;
        Ogre::RenderSystem  *mRenderSystem;

        Ogre::CompositorWorkspace *mWorkspaces[2];
        Ogre::SceneNode     *mCamerasNode;
        std::shared_ptr<PoseState> mCameraPoseState;

        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
        Ogre::Matrix4           mDevicePose[vr::k_unMaxTrackedDeviceCount];
        vr::ETextureType        mApiTextureType;
        Ogre::Vector3           mCullCameraOffset;

        VrWaitingMode::VrWaitingMode mWaitingMode;
        VrWaitingMode::VrWaitingMode mFirstGlitchFreeMode;

        bool                    mMustSyncAtEndOfFrame;

        int mFrameCnt;
        bool mTrackPose;
        bool mWriteTexture;

        void syncCamera(void);
        void updateHmdTrackingPose(void);
//      TODO: use these functions, when we go to moving camera
//         void syncCullCamera(void);
    public:
        OpenVRCompositorListener(
            vr::IVRSystem *hmd, vr::IVRCompositor *vrCompositor,
            Ogre::TextureGpu *vrTexture, Ogre::Root *root,
            Ogre::CompositorWorkspace *workspaces[2],
            Ogre::SceneNode *mCamerasNode,
            std::shared_ptr<PoseState> poseState );
        virtual ~OpenVRCompositorListener();

        virtual bool frameStarted( const Ogre::FrameEvent& evt );
        virtual bool frameRenderingQueued( const Ogre::FrameEvent &evt );
        virtual bool frameEnded( const Ogre::FrameEvent& evt );

        virtual void workspacePreUpdate( Ogre::CompositorWorkspace *workspace );
        virtual void passPreExecute( Ogre::CompositorPass *pass );

        virtual void passSceneAfterShadowMaps( Ogre::CompositorPassScene *pass );
        virtual void passSceneAfterFrustumCulling( Ogre::CompositorPassScene *pass );

        /// See VrWaitingMode::VrWaitingMode
        void setWaitingMode( VrWaitingMode::VrWaitingMode waitingMode );
        VrWaitingMode::VrWaitingMode getWaitingMode(void)   { return mWaitingMode; }

        void triggerWriteTexture(){mWriteTexture = true;};
        int getFrameCnt(void){return mFrameCnt;};

        /** When operating in VrWaitingMode::AfterSceneGraph or later, there's a chance
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
        void setGlitchFree( VrWaitingMode::VrWaitingMode firstGlitchFreeMode );

        /** Returns true if the current waiting mode can update the camera immediately,
            false if it must wait until the end of the frame.
        @remarks
            VrWaitingMode::BeforeSceneGraph and earlier always returns true.

            See OpenVRCompositorListener::setGlitchFree
        */
        bool canSyncCameraTransformImmediately(void) const;

    };
}

#endif
