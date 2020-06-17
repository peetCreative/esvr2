#ifndef _Esvr2_GameState_H_
#define _Esvr2_GameState_H_

#include "Esvr2StereoRendering.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2PointCloud.h"

#include "OgrePrerequisites.h"
#include "TutorialGameState.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"

namespace esvr2
{
    class GameState : public Demo::TutorialGameState
    {
        GraphicsSystem *mStereoGraphicsSystem;

        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::ManualObject       *mProjectionRectangle[4];
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

        Ogre::Real mProjPlaneDistance,
            mLeft[4], mRight[4], mTop[4], mBottom[4];
        Ogre::Real mScale;
        Ogre::String mTextureName[2], mDatablockName[2];
        void createProjectionPlanes();
        void createTooltips();
        void createPointCloud();
        void createMesh();

    public:
        GameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData );
        void calcAlign( StereoCameraConfig &cameraConfig,
                        Ogre::Real projPlaneDistance = 1.0f);
        void createScene01(void);

        void update( float timeSinceLast );
        void keyReleased( const SDL_KeyboardEvent &arg );

        void _notifyStereoGraphicsSystem(
            GraphicsSystem *stereoGraphicsSystem )
        {
            mStereoGraphicsSystem = stereoGraphicsSystem;
            _notifyGraphicsSystem( stereoGraphicsSystem );
        }
    };
}

#endif
