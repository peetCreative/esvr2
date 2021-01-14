#ifndef _Esvr2_GameState_H_
#define _Esvr2_GameState_H_

#include "Esvr2.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2PointCloud.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2InteractiveElement2DDef.h"

#include "OgrePrerequisites.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"
#include "OgreRectangle2D2.h"

namespace esvr2
{
    //Define what is modified when the mouse is moved
    typedef enum {
        MM_NONE,
        MM_ORIENTATION,
        MM_TRANSLATION
    } MouseManipulationType;

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
        Esvr2                       *mEsvr2;
        GraphicsSystem              *mGraphicsSystem;

//        HmdConfig                   mHmdConfig;
        StereoCameraConfig          mCameraConfig;

        Ogre::SceneNode             *mVRCamerasNode;
        Ogre::SceneNode             *mLaparoscopeCamerasNode;

        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::HlmsUnlitDatablock *mInfoScreenDatablock;
        Ogre::Rectangle2D        *mProjectionRectangle2D;
        Ogre::ManualObject       *mVRInfoScreen;
        Ogre::ManualObject       *mProjectionRectangle[4];
        Ogre::ManualObject       *mAxis;
        Ogre::ManualObject       *mAxisCameras;
        Ogre::ManualObject       *mAxisVROrigin;
        Ogre::ManualObject       *mAxisVRCameras;

        Ogre::v1::BillboardSet   *mTooltips;
        PointCloud               *mPointCloud;
        Ogre::SceneNode          *mVRSceneNodeLight;
        Ogre::SceneNode          *mVRSceneNodeMesh;
        Ogre::SceneNode          *mVRSceneNodeProjectionPlanesOrigin;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRaw;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRect;
        Ogre::SceneNode          *mLaparoscopeSceneNodePointCloud;
        Ogre::SceneNode          *mLaparoscopeSceneNodeTooltips;
        Ogre::SceneNode          *mInfoScreenSceneNode;

        const Ogre::Vector2      mInfoScreenDim = Ogre::Vector2(1.28f, 0.72f);
        bool                     mIntersectsInfoScreen;
//        UIStatusType             mUIStatus;
        bool                     mIsUIVisible;
        InteractiveElement2DPtr  mHoverUIElement;
        bool                     mUIActive;
        InteractiveElement2DPtr  mActiveUIElement;
        Ogre::Vector2            mInfoScreenUV;
        bool                     mIsStereo;
        size_t                   mEyeNum;

        bool mDisplayHelpMode;
        Ogre::String mDebugText;
        Ogre::String mHelpDescription;
        InteractiveElement2DList mInteractiveElement2DList;

        Ogre::v1::PanelOverlayElement *mViewingDirectionIndicator;

        Ogre::Real mProjPlaneDistance[4],
            mLeft[4], mRight[4], mTop[4], mBottom[4];
        Ogre::Real mScale;
        Ogre::String mTextureName[2], mDatablockName[2];

        bool mWindowHasFocus;
        bool mMouseInWindow;

        bool mGrabPointer, mIsMouseRelative, mWrapPointerManually, mWarpCompensate;

        int mWarpX, mWarpY;
        bool mWantRelative, mWantMouseVisible;
        MouseManipulationType mMouseManipulate;
        Ogre::Real mVRCameraNodeYaw, mVRCameraNodePitch;
        Ogre::Real mVRCameraNodeTransZ, mVRCameraNodeTransX;

        void createProjectionRectangle2D();
        void createVRProjectionPlanes();
        Ogre::ManualObject *createAxisIntern( Ogre::SceneManager *sceneManager );
        void createLaparoscopeAxisCamera(void );
        void createVRAxisCamera(void);
        void createTooltips();
        void createPointCloud();
        void createVROverlays(void);
        void addViewDirectionIndicator();
        void createMesh();

        InteractiveElement2DPtr findInteractiveElement2DByName(Ogre::String);
        InteractiveElement2DPtr findInteractiveElement2DByUV(
                Ogre::Vector2 uv);
        void generateDebugText(
                Ogre::uint64 microSecsSinceLast, Ogre::String &outText );

        void updateOverlayElements();
        void toggleUI();
        void holdUI(Ogre::uint64 timeSinceLast);

        bool setDistortion(Distortion dist);


        void createVRCamerasNodes();
        void createLaparoscopeCameraNodes();

        void updateLaparoscopePoseFromPoseState();

        void setMouseRelative( bool relative );
        void setMouseVisible( bool visible );
        void updateMouseSettings(void);
        void mouseMovedRelative(const SDL_Event &arg);
        void mousePressed( const SDL_MouseButtonEvent &arg );
        void mouseReleased( const SDL_MouseButtonEvent &arg );
        void warpMouse( int x, int y );
        void wrapMousePointer( const SDL_MouseMotionEvent& evt );

        bool keyPressed( const SDL_KeyboardEvent &arg );
        bool keyReleased( const SDL_KeyboardEvent &arg );

        void updateVRCamerasNode(void);
        void calcAlign( bool center);

        void createVRInfoScreen(void);

        void createVRFloor();

        void readHeadGestures();
    public:

        void update( Ogre::uint64 microSecsSinceLast );

        //TODO: implement destroyScene
        void destroyScene() {};
        void handleSdlEvent( const SDL_Event& evt );

        Ogre::Quaternion getHeadOrientation();
        Ogre::Quaternion getProjectionPlanesOrientation();
        Ogre::Vector3 getHeadPosition();

        void setDebugText(Ogre::String debugText);
        void createInteractiveElement2D(
                Ogre::String defName,
                const boost::function<void()> &togglecb,
                const boost::function<void(Ogre::uint64)> &holdcb);

        void addInteractiveElement2D(InteractiveElement2DPtr interactiveElement2D);
        void adjustToHeadHight();
    };
}

#endif
