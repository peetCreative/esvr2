
#ifndef _Demo_StereoRenderingGraphicsSystem_H_
#define _Demo_StereoRenderingGraphicsSystem_H_

#include "StereoRendering.h"

#include "GraphicsSystem.h"
#include "OgreSceneNode.h"
#include "OgreCamera.h"
#include "Compositor/OgreCompositorManager2.h"

namespace Demo
{
    class StereoGraphicsSystem : public GraphicsSystem
    {
        Ogre::SceneNode             *mCamerasNode;
        Ogre::Camera                *mEyeCameras[2];
        Ogre::CompositorWorkspace   *mEyeWorkspaces[2];
        WorkspaceType               mWorkSpaceType;

        //-------------------------------------------------------------------------------
        virtual void createCamera(void);

        virtual Ogre::CompositorWorkspace* setupCompositor();

    public:
        StereoGraphicsSystem( GameState *gameState, WorkspaceType wsType );
        ~StereoGraphicsSystem();
    };
}

#endif
