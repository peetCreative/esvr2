#ifndef _Esvr2_GameState_H_
#define _Esvr2_GameState_H_

#include "Esvr2.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2Controller.h"
#include "Esvr2PointCloud.h"
#include "Esvr2InteractiveElement.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2InteractiveElement2DDef.h"
#include "Esvr2FootPedal.h"
#include "Esvr2SettingsEventLog.h"

#include "OgrePrerequisites.h"

#include "OgreCamera.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"
#include "OgreRectangle2D2.h"

#define MENU_GENERAL "GeneralMenu"
#define MENU_DEBUG "MenuDebug"
#define MENU_DISTANCE "MenuDistance"
#define MENU_MOVE "MenuMove"
#define MENU_ADJUST_TO_HEAD "MenuAdjustToHead"
#define MENU_CHANGE_DISTORTION "MenuDistortion"
#define MENU_CHANGE_CONTROLLER "MenuController"
#define MENU_SETUP1 "Setup1"
#define MENU_SETUP_CURSOR "SetupNext"
#define MENU_SETUP_MOVE "SetupMove"
#define MENU_SETUP_DISTANCE "SetupDistance"
#define MENU_SETUP_EXPLANATION "SetupExplanation"

namespace esvr2
{
    //Define what is modified when the mouse is moved
    typedef enum {
        MM_NONE,
        MM_ORIENTATION,
        MM_TRANSLATION,
        MM_ROLL
    } MouseManipulationType;

    enum UIStatusType {
        UI_NONE,
        UI_GENERAL,
        UI_CONTROLLER,
        UI_CONTROLLER_ACTIVE
    };

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
        ControllerPtr               mCurrentController = nullptr;
        ControllerPtr               mOpt0Controller = nullptr;
        ControllerPtr               mOpt1Controller = nullptr;
        ControllerPtr               mOpt2Controller = nullptr;

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
        Ogre::SceneNode          *mVRSceneNodeLight;
        Ogre::SceneNode          *mVRSceneNodeMesh;
        Ogre::SceneNode          *mVRSceneNodeProjectionPlanesOrigin;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRaw;
        Ogre::SceneNode          *mVRSceneNodesProjectionPlaneRect;
        Ogre::SceneNode          *mLaparoscopeSceneNodeTooltips;
        Ogre::SceneNode          *mInfoScreenStaticSceneNode = nullptr;
        Ogre::SceneNode          *mInfoScreenSceneNode = nullptr;
        Ogre::SceneNode          *mFloorSceneNode {nullptr};

        UIStatusType             mUIStatus = UI_NONE;
        bool                     mIsUIVisible;
        Ogre::IdString           mUIStatusStr = Ogre::IdString();
        InteractiveElement2DPtr  mHoverUIElement = nullptr;
        InteractiveElementPtr    mBackgroundUIElement = nullptr;
        bool                     mUIActive;
        InteractiveElementPtr    mActiveUIElement;
        Ogre::Vector2            mInfoScreenUV;
        bool                     mIsStereo;
        size_t                   mEyeNum;

        bool mDisplayHelpMode;
        Ogre::String mDebugText;
        Ogre::String mHelpDescription;
        InteractiveElement2DList mInteractiveElement2DList;
        InteractiveElementPtr mAdjustToHeadHightIE = nullptr;
        InteractiveElementPtr mAdjustProjectionPlaneDistanceIE = nullptr;
        InteractiveElementPtr mMoveIE = nullptr;
        InteractiveElementPtr mSetupStartIE = nullptr;
        InteractiveElementPtr mVoidIE = nullptr;
        InteractiveElement2DPtr mInfoBoxDistortionSelect = nullptr;
        InteractiveElement2DPtr mInfoBoxControllerSelect = nullptr;

        Ogre::v1::PanelOverlayElement *mViewingDirectionIndicator;
        bool mShowViewingDirectionIndicator = false;

        //correct Distance
        Ogre::Real mCorrectProjPlaneDistance[4],
            mLeft[4], mRight[4], mTop[4], mBottom[4];
        Ogre::Real mScale;
        Ogre::String mTextureName[2], mDatablockName[2];

        Ogre::Real mAdjustProjectionPlaneInitialPitch = 0;
        Ogre::Real mAdjustProjectionPlaneRawInitialDistance = 0;
        Ogre::Real mAdjustProjectionPlaneRectInitialDistance = 0;
        const Ogre::Real mAdjustProjectionPlaneFact = 5.0f;


        bool mWindowHasFocus;
        bool mMouseInWindow;

        bool mGrabPointer, mIsMouseRelative, mWrapPointerManually, mWarpCompensate;

        int mWarpX, mWarpY;
        bool mWantRelative, mWantMouseVisible;
        MouseManipulationType mMouseManipulate;
        Ogre::Real mVRCameraNodeYaw, mVRCameraNodePitch;
        Ogre::Real mVRCameraNodeTransZ, mVRCameraNodeTransX;
        Ogre::Real mVRCameraNodeRoll;

        SettingsEventLogList mSettingsEventLogs;

        void createProjectionRectangle2D();
        void createVRProjectionPlanes();
        Ogre::ManualObject *createAxisIntern( Ogre::SceneManager *sceneManager );
        void createLaparoscopeAxisCamera(void );
        void createVRAxisCamera(void);
        void createTooltips();
        void createVROverlays(void);
        void addViewDirectionIndicator();
        void createMesh();

        InteractiveElement2DPtr findInteractiveElement2DByName(Ogre::String);
        InteractiveElement2DPtr findInteractiveElement2DByUV(
                Ogre::Vector2 uv);
        void generateDebugText(
                Ogre::uint64 microSecsSinceLast, Ogre::String &outText );

        void updateOverlayElements();
        void setController(ControllerType ct);
        void toggleUIPress();
        void holdUI(Ogre::uint64 currentTimeMs);
        void toggleUIRelease();

        bool setDistortion(Distortion dist);

        void createControllers();

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

        void goToMenu(
                Ogre::IdString menu,
                InteractiveElementPtr backgroundIE);

        void resetProjectionPlaneDistance();
        void initAdjustProjectionPlaneDistance();
        void holdAdjustProjectionPlaneDistance(Ogre::uint64 time);

        void moveScreenInit();

        void setupStart();
        void setupFinish();

#ifdef USE_POINTCLOUD
        PointCloud               *mPointCloud {nullptr};
        Ogre::SceneNode          *mLaparoscopeSceneNodePointCloud {nullptr};
        void createPointCloud();
#endif
    public:

        void update( Ogre::uint64 msSinceLast );

        //TODO: implement destroyScene
        void destroyScene() {};
        void handleSdlEvent( const SDL_Event& evt );
#ifdef USE_FOOTPEDAL
        void handleFootPedalEvent( const FootPedalEvent& evt );
#endif

        Ogre::Quaternion getHeadOrientation();
        Ogre::Quaternion getProjectionPlanesOrientation();
        Ogre::Vector3 getHeadPosition();
        bool isHeadPositionCentered();


        void setDebugText(Ogre::String debugText);
        InteractiveElement2DPtr createInteractiveElement2D(
                Ogre::String defName,
                std::vector<Ogre::IdString> menus,
                Ogre::String text = "");

        void addInteractiveElement2D(InteractiveElement2DPtr interactiveElement2D);
        void adjustToHeadHight();
        void moveScreen(Ogre::uint64 time);

        void addSettingsEventLog(Ogre::String eventStr);
    };
}

#endif
