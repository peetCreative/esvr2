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
#include "OgreRectangle2D2.h"

namespace esvr2
{
    class GameState : public Demo::TutorialGameState
    {
        GraphicsSystem *mStereoGraphicsSystem;

        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::Rectangle2D        *mProjectionRectangle2D;
        Ogre::ManualObject       *mProjectionRectangle[4];
        Ogre::ManualObject       *mAxis;
        Ogre::ManualObject       *mAxisCameras;
        Ogre::v1::BillboardSet   *mTooltips;
        PointCloud               *mPointCloud;
        Ogre::SceneNode          *mSceneNodeLight;
        Ogre::SceneNode          *mSceneNodeCamera[2];
        Ogre::SceneNode          *mSceneNodePointCloud;
        Ogre::SceneNode          *mSceneNodeTooltips;
        Ogre::SceneNode          *mSceneNodeMesh;
        Ogre::VrData             *mVrData;
        bool                     mIsStereo;
        size_t                   mEyeNum;

        Ogre::Real mProjPlaneDistance[4],
            mLeft[4], mRight[4], mTop[4], mBottom[4];
        Ogre::Real mScale;
        Ogre::String mTextureName[2], mDatablockName[2];
        void createProjectionRectangle2D();
        void createProjectionPlanes();
        Ogre::ManualObject *createAxisIntern( void );
        void createAxis( void );
        void createTooltips();
        void createPointCloud();
        void createMesh();

    public:
        GameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData );
        void calcAlign( );
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
