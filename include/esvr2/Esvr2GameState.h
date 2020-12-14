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
    friend OpenVRCompositorListener;
    public:
        GameState( Esvr2 *esvr2, GraphicsSystem *graphicsSystem );

        void loadDatablocks();
        void createLaparoscopeScene(void);
        void createVRScene(void);

    private:
    Esvr2                           *mEsvr2;
        GraphicsSystem              *mGraphicsSystem;

        Ogre::VrData                mVrData;
        HmdConfig                   mHmdConfig;
        StereoCameraConfig          mCameraConfig;

        Ogre::v1::OverlaySystem     *mOverlaySystem;

        Ogre::SceneNode             *mVRCamerasNode;
        Ogre::SceneNode             *mLaparoscopeCamerasNode;

        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::Rectangle2D        *mProjectionRectangle2D;
        Ogre::ManualObject       *mProjectionRectangle[4];
        Ogre::ManualObject       *mAxis;
        Ogre::ManualObject       *mAxisCameras;
        Ogre::ManualObject       *mAxisVROrigin;
        Ogre::ManualObject       *mAxisVRCameras;

        Ogre::v1::BillboardSet   *mTooltips;
        PointCloud               *mPointCloud;
        Ogre::SceneNode          *mVRSceneNodeLight;
        Ogre::SceneNode          *mVRSceneNodeMesh;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRaw;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRect;
        Ogre::SceneNode          *mLaparoscopeSceneNodePointCloud;
        Ogre::SceneNode          *mLaparoscopeSceneNodeTooltips;
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
        Ogre::ManualObject *createAxisIntern( Ogre::SceneManager *sceneManager );
        void createAxis( void );
        void createVRAxis(void);
        void createTooltips();
        void createPointCloud();
        void createVROverlays(void);
        void createMesh();

        void generateDebugText(
                Ogre::uint64 microSecsSinceLast, Ogre::String &outText );

        void createVRCamerasNodes();
        void createLaparoscopeCameraNodes();

        void updateLaparoscopePoseFromPoseState();
    public:
        void calcAlign( );

        void update( Ogre::uint64 microSecsSinceLast );

        //TODO: implement destroyScene
        void destroyScene() {};
//        void keyReleased( const SDL_KeyboardEvent &arg );

    };
}

#endif
