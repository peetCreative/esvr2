
#ifndef _Demo_StereoRenderingGameState_H_
#define _Demo_StereoRenderingGameState_H_

#include "StereoRendering.h"

#include "OgrePrerequisites.h"
#include "TutorialGameState.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"

namespace Demo
{
    class StereoRenderingGameState : public TutorialGameState
    {
        Ogre::SceneNode          *mSceneNodeCube;
        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::ManualObject       *mProjectionRectangle[2];
        Ogre::SceneNode          *mSceneNodeCamera;
        Ogre::SceneNode          *mSceneNodeLight;
        Ogre::VrData             *mVrData;
        bool                     mIsStereo;
        size_t                   mEyeNum;

        Ogre::Real mProjPlaneDistance, mLeft[2], mRight[2], mTop[2], mBottom[2];
        Ogre::String mTextureName[2], mDatablockName[2];
        void createCube();
        void createProjectionPlanes();

    public:
        StereoRenderingGameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData );
        void calcAlign( CameraConfig &cameraConfig,
                        Ogre::Real projPlaneDistance = 1.0f);
        virtual void createScene01(void);

        virtual void update( float timeSinceLast );
    };
}

#endif
