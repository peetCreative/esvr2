#ifndef _Esvr2_GameState_H_
#define _Esvr2_GameState_H_

#include "Esvr2.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2PointCloud.h"

#include "OgrePrerequisites.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"
#include "OgreRectangle2D2.h"
#include "Overlay/OgreTextAreaOverlayElement.h"

#include <memory>

namespace esvr2
{
    class GameState
    {
    friend GraphicsSystem;
    public:
        GameState( Esvr2 *esvr2 );

        void _notifyStereoGraphicsSystem(
            std::shared_ptr<GraphicsSystem> graphicsSystem )
        {
            mGraphicsSystem = graphicsSystem;
        }

        void createScene01(void);

    private:
        Esvr2* mEsvr2;
        std::shared_ptr<GraphicsSystem> mGraphicsSystem;

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
        bool                     mIsStereo;
        size_t                   mEyeNum;

        bool mDisplayHelpMode;
        Ogre::v1::TextAreaOverlayElement* mDebugText;
        Ogre::v1::TextAreaOverlayElement* mDebugTextShadow;
        Ogre::String mHelpDescription;

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

        void generateDebugText( float timeSinceLast, Ogre::String &outText );


    public:
        void calcAlign( );

        void update( float timeSinceLast );
        //TODO: implement
        void destroyScene() {};
//        void keyReleased( const SDL_KeyboardEvent &arg );

    };
}

#endif
