#ifndef _Demo_StereoRenderingGameState_H_
#define _Demo_StereoRenderingGameState_H_

#include "StereoRendering.h"
#include "StereoRenderingGraphicsSystem.h"
#include "PointCloud.h"

#include "OgrePrerequisites.h"
#include "TutorialGameState.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"

namespace esvr2
{
    class StereoRenderingGameState : public Demo::TutorialGameState
    {
        StereoGraphicsSystem *mStereoGraphicsSystem;

        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::ManualObject       *mProjectionRectangle[2];
        Ogre::v1::BillboardSet   *mTooltips;
        PointCloud               *mPointCloud;
        Ogre::SceneNode          *mSceneNodeLight;
        Ogre::SceneNode          *mSceneNodeCamera;
        Ogre::SceneNode          *mSceneNodePointCloud;
        Ogre::SceneNode          *mSceneNodeTooltips;
        Ogre::SceneNode          *mSceneNodeMesh;
        Ogre::VrData             *mVrData;
        bool                     mIsStereo;
        size_t                   mEyeNum;

        Ogre::Real mProjPlaneDistance, mLeft[2], mRight[2], mTop[2], mBottom[2];
        Ogre::Real mScale;
        Ogre::String mTextureName[2], mDatablockName[2];
        void createProjectionPlanes();
        void createTooltips();
        void createPointCloud();
        void createMesh();

    public:
        StereoRenderingGameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData );
        void calcAlign( StereoCameraConfig &cameraConfig,
                        Ogre::Real projPlaneDistance = 1.0f);
        void createScene01(void);

        void update( float timeSinceLast );
        void keyReleased( const SDL_KeyboardEvent &arg );

        void _notifyStereoGraphicsSystem(
            StereoGraphicsSystem *stereoGraphicsSystem )
        {
            mStereoGraphicsSystem = stereoGraphicsSystem;
            _notifyGraphicsSystem( stereoGraphicsSystem );
        }
    };
}

#endif
