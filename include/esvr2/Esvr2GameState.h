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
    //! \brief Define what is modified when the mouse is moved
    typedef enum {
        MM_NONE,
        MM_ORIENTATION,
        MM_TRANSLATION,
        MM_ROLL
    } MouseManipulationType;

    //! \brief State of the Menü/Cameracontroller
    enum UIStatusType {
        UI_NONE,
        UI_GENERAL,
        UI_CONTROLLER,
        UI_CONTROLLER_ACTIVE
    };

    //! \brief State and Management of the virtual Objects
    class GameState
    {
    friend GraphicsSystem;
    friend OpenVRCompositorListener;
    public:
        /*! \brief Constructor of GameState
         * \param esvr2 ptr to Esvr2 Instance
         * \param graphicsSystem ptr to GraphicsSystem Instance
         */
        GameState( Esvr2 *esvr2, GraphicsSystem *graphicsSystem );

        //! \brief create Ogre-Datablock Structures
        /* ! create using Ogre-Hlms Datablocks for colors and
         * storage space for the projection place
         */
        void loadDatablocks();
        //! \brief create all the objects which makeup the virtual Laparoscope scene
        /*! This scene should enable overlays using  virtual pointclouds or objects as
         * tracked tooltips and axis marking a checker board.
         * ATM it is mostly used for the menu where 2D-Overlays are placed on top.
         */
        void createLaparoscopeScene(void);
        //! \brief create all the objects which makeup the virtual Laparoscope scene
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

        //! correct Distance
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

        /*! \brief create the virtual projection plane
         */
        void createVRProjectionPlanes();
        //! \brief creates a virtual axis object
        /* \param sceneManager ptr to the corresponding sceneManager Object
         * \return ptr to the created object
         */
        Ogre::ManualObject *createAxisIntern( Ogre::SceneManager *sceneManager );
        //! \brief create virtual axis at the origin in the laparoscopic images
        /* needs PoseState Component
         * \deprecated kind of
         */
        void createLaparoscopeAxisCamera(void );
        //! \brief create virtual axis at the Cameraposition in the VR Scene
        /* \deprecated not used
         */
        void createVRAxisCamera(void);
        //! \brief create some points for tooltips in the image
        /*! \deprecated
         */
        void createTooltips();
        // \brief create 2D-overlay objects in the Laparoscopic Scene for Menu
        void createVROverlays(void);
        //! \brief add small red dot in the menu
        void addViewDirectionIndicator();
        //! \brief create a cube for debugging in the vr scene
        void createMesh();

        //! \brief gets an registered InteractiveElement by it's name
        InteractiveElement2DPtr findInteractiveElement2DByName(Ogre::String);
        //! \brief gets an registered InteractiveElement by it's location on the menuplane
        InteractiveElement2DPtr findInteractiveElement2DByUV(
                Ogre::Vector2 uv);
        //! \brief updates the text in the menu debug information
        void generateDebugText(
                Ogre::uint64 microSecsSinceLast, Ogre::String &outText );

        //! \brief updates the apperance of the InteractiveElements
        void updateOverlayElements();
        //! \brief change the laparoscope Controller
        /*!
         * @param ct controllerType to change to
         */
        void setController(ControllerType ct);
        //! \brief toggles press on the currently focused InteractiveElement
        void toggleUIPress();
        //! \brief activatetes hold on the currently focused InteractiveElement
        /*!
         * @param currentTimeMs time of the last update
         */
        void holdUI(Ogre::uint64 currentTimeMs);
        //! \brief toggles release on the current focused InteractiveElement
        void toggleUIRelease();

        //! \brief set the targeted Distortion
        bool setDistortion(Distortion dist);

        //! \brief create the available LaparoscopeController
        void createControllers();

        //! \brief create SceneNodes in the VR-Scene for the virtual cameras
        void createVRCamerasNodes();
        //! \brief create SceneNodes in the virtual Laparoscope-Scene for the virtual cameras
        void createLaparoscopeCameraNodes();

        /*! \brief takes the Pose of the laparoscope center from the PoseState Component
         * and updates the view of the laparoscopic scene
         */
        void updateLaparoscopePoseFromPoseState();

        void setMouseRelative( bool relative );
        void setMouseVisible( bool visible );
        void updateMouseSettings(void);
        //! \brief callback when mouse is moved update move VR CameraNode
        void mouseMovedRelative(const SDL_Event &arg);
        //! \brief callback when mouse is clicked update move VR CameraNode
        /*! *left click adjust the orientation
         * *right click moves forward/backward
         * *middle rolls to left/right
         */
        void mousePressed( const SDL_MouseButtonEvent &arg );
        //! \brief callback for mouse released
        void mouseReleased( const SDL_MouseButtonEvent &arg );
        //! \brief callback if mouse goes out of window to place it back
        //! copied from Ogre examples
        void warpMouse( int x, int y );
        //! \brief kind of similar to above
        //! copied from Ogre examples
        void wrapMousePointer( const SDL_MouseMotionEvent& evt );

        //! \brief callback when a keyboard key is pressed
        /*! - m activate _menu_ and click/hold menu InteractiveElements
         * - n activate the current laparocsope controller and click/hold InteractiveElements
         * @param arg the keyboard event
         * @return success
         */
        bool keyPressed( const SDL_KeyboardEvent &arg );
        //! \brief callback when a keyboard key is released
        /*! - m activate _menu_ and click menu InteractiveElements
         * - n activate the current laparocsope controller
         * @param arg the keyboard event
         * @return success
         */
        bool keyReleased( const SDL_KeyboardEvent &arg );

        //! \brief rotates and moves camera when mouse was left/right clicked
        void updateVRCamerasNode(void);
        //! \brief calculate the position of the projection plane
        /* \param center center the projection plane in the camera calibration camera matrix
         */
        void calcAlign( bool center);

        //! \brief create the virtual projection plane for the Menu
        void createVRInfoScreen(void);
        //! \brief create the floor of the VR-Scene
        void createVRFloor();

        //! \brief calculates from the SceneNode of the virtual VR-camera the focused point on the menuplane
        void readHeadGestures();

        //! \brief callback to change the menu
        /*!
         * @param menu MenuID
         * @param backgroundIE the interactive Element which is activated, when no InteractiveElement is not selected
         */
        void goToMenu(
                Ogre::IdString menu,
                InteractiveElementPtr backgroundIE);

        //! \brief resets the distance to the projection plane to the calculated correct value
        void resetProjectionPlaneDistance();
        //! \brief init callback for functionality Adjust to Projection Plane
        void initAdjustProjectionPlaneDistance();
        //! \brief hold callback for functionality Adjust to Projection Plane
        /*!
         * @param time since the last update
         */
        void holdAdjustProjectionPlaneDistance(Ogre::uint64 time);
        //! \brief hold callback for functionality Adjust to Projection Plane
        /*!
         * @param time since the last update
         */
        void releaseAdjustProjectionPlaneDistance();

        //! \brief called when start position is called
        /*! We adjust the position of the Projection plane to the correct distance
         * makes sure at the end it feels correct.
         */
        void moveScreenInit();
        //! \brief called when Projectionplane moving stops
        /*! add eventlog and add to cache
         */
        void moveScreenStop();

        //! \brief setup the first info menu
        /*! attach the menu plane to the head SceneNode
         */
        void setupStart();
        //! \brief callback when the setup is finished
        void setupFinish();

#ifdef USE_POINTCLOUD
        PointCloud               *mPointCloud {nullptr};
        Ogre::SceneNode          *mLaparoscopeSceneNodePointCloud {nullptr};
        void createPointCloud();
#endif
    public:
        //! \brief update callback for the scenes
        void update( Ogre::uint64 msSinceLast );

        //TODO: implement destroyScene
        void destroyScene() {};
        //! \brief callback for incomming IO (SDL) Events
        /*! Handles: Mouse Motion, Mouse click events, Keyboard events
         */
        void handleSdlEvent( const SDL_Event& evt );
#ifdef USE_FOOTPEDAL
        void handleFootPedalEvent( const FootPedalEvent& evt );
#endif
        //! \brief get the orientation of the VR-Headset
        Ogre::Quaternion getHeadOrientation();
        //! \brief get the orientation of the Projection Plane
        Ogre::Quaternion getProjectionPlanesOrientation();
        //! \brief get the Position of the VR-Headset
        Ogre::Vector3 getHeadPosition();
        //! \brief checks if the VR-Headset is close to the initial configured one
        /*!
         *
         * @return return if the current position VR-Headset position is close (threshold `mConfig->centerEpsilon`) to the initial configured
         */
        bool isHeadPositionCentered();


        //! \brief change DebugText
        void setDebugText(Ogre::String debugText);
        //! \brief creates a 2D-overlay object
        /*! Creates an a 2D-overlay object in the laparoscopic scene for the menu
         *  hereby the look is defined by InteractiveElementsDef.yaml
         *  \param defName name of an object, which was configured in InteractiveElementsDef.yaml
         *  \param menus list of menu names, when the overlay should be visible
         *  \param text the text in the overlay
         *  \return ptr to the interactiveElement
         */
        InteractiveElement2DPtr createInteractiveElement2D(
                Ogre::String defName,
                std::vector<Ogre::IdString> menus,
                Ogre::String text = "");

        //! \brief adds interactiveElement2D to the list of interactive elements
        /*! @param interactiveElement2D to be added interactiveElement
         */
        void addInteractiveElement2D(InteractiveElement2DPtr interactiveElement2D);
        //! \brief adjusts the height of the virtual projection plane to the height of the tracked head
        void adjustToHeadHight();
        //! \brief moves the Menu Projectionplane to the focus point
        /*! @param time since the last update
         */
        void moveScreen(Ogre::uint64 time);

        //! \brief add a line in the Log of the settings
        /*! write out PorjectionPlane Distance, Orientation, Distortion
         * and a string
         * @param eventStr String to add to this event
         */
        void addSettingsEventLog(Ogre::String eventStr);
    };
}

#endif
