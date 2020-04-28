
#ifndef _Demo_StereoRenderingGraphicsSystem_H_
#define _Demo_StereoRenderingGraphicsSystem_H_

#include "StereoRendering.h"

#include "GraphicsSystem.h"
#include "OgreSceneNode.h"
#include "OgreCamera.h"
#include "Compositor/OgreCompositorManager2.h"

#include "openvr.h"


namespace Demo
{
    class OpenVRCompositorListener;

    class StereoGraphicsSystem : public GraphicsSystem
    {
    private:
        //Depending on this type start with different Compositor setup
        WorkspaceType               mWorkSpaceType;

        Ogre::SceneNode             *mCamerasNode;
        //two real cameras and two workspaces (two cameras rendering) or
        //only use one VR Camera and workspace (Instanced Rendering)
        Ogre::Camera                *mEyeCameras[2];
        Ogre::CompositorWorkspace   *mVrWorkspaces[2];
        //but there is never the less only one cull camera
        Ogre::CompositorWorkspace   *mMirrorWorkspace;
        Ogre::Camera                *mVrCullCamera;
        Ogre::TextureGpu            *mVrTexture;

        OpenVRCompositorListener    *mOvrCompositorListener;

        // these are nullptr if there is no SteamVR available through OpenVR
        vr::IVRSystem *mHMD;
        vr::IVRCompositor *mVRCompositor;
        std::string mStrDriver;
        std::string mStrDisplay;
        std::string mDeviceModelNumber;
        vr::TrackedDevicePose_t mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];

        //-------------------------------------------------------------------------------
        virtual void createCamera(void);
        virtual Ogre::CompositorWorkspace* setupCompositor(void);

        std::string GetTrackedDeviceString(
            vr::TrackedDeviceIndex_t unDevice,
            vr::TrackedDeviceProperty prop,
            vr::TrackedPropertyError *peError = nullptr);
        void initCompositorVR(void);
        void initOpenVR(void);
    public:
        StereoGraphicsSystem( GameState *gameState, WorkspaceType wsType );
        ~StereoGraphicsSystem();
    };
}

#endif
