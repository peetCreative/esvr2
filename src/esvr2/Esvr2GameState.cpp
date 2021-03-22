#include "Esvr2GameState.h"

#include "Esvr2.h"
#include "Esvr2PointCloud.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2Helper.h"
#include "Esvr2Opt0Controller.h"
#include "Esvr2Opt1Controller.h"
#include "Esvr2Opt2Controller.h"
#include "Esvr2SettingsEventLog.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"
#include "OgreMaterialManager.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsUnlit.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreRectangle2D2.h"
#include "OgreManualObject2.h"
#include "OgreBillboard.h"
#include "OgreEntity.h"
#include "OgreFrameStats.h"
#include "OgreMeshManager2.h"
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreOverlayManager.h"
#include "Overlay/OgreTextAreaOverlayElement.h"
#include "Overlay/OgreBorderPanelOverlayElement.h"

#include <boost/bind.hpp>
#include "SDL.h"

namespace esvr2
{
    GameState::GameState(Esvr2 *esvr2, GraphicsSystem *graphicsSystem):
            mEsvr2(esvr2),
            mGraphicsSystem( graphicsSystem ),
            mVideoDatablock{ nullptr, nullptr },
            mInfoScreenDatablock(nullptr),
            mVRInfoScreen(nullptr),
            mProjectionRectangle{ nullptr, nullptr, nullptr, nullptr },
            mAxis( nullptr ),
            mAxisCameras( nullptr ),
            mTooltips( nullptr ),
            mVRSceneNodeLight( nullptr ),
            mVRSceneNodeMesh( nullptr ),
            mVRSceneNodeProjectionPlanesOrigin(nullptr),
            mVRSceneNodesProjectionPlaneRaw(nullptr),
            mVRSceneNodesProjectionPlaneRect(nullptr),
            mLaparoscopeSceneNodeTooltips( nullptr ),
            mIsUIVisible(false),
            mHoverUIElement(nullptr),
            mUIActive(false),
            mActiveUIElement(nullptr),
            mInfoScreenUV( 0, 0 ),
            mIsStereo( esvr2->mConfig->isStereo ),
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 ),
            mCorrectProjPlaneDistance{0, 0, 0, 0 },
            mLeft{ 0, 0 },
            mRight{ 0, 0 },
            mTop{ 0, 0 },
            mBottom{ 0, 0 },
            mScale( 1.0f ),
            //Window
            mWindowHasFocus(false),
            mMouseInWindow(false),
            mGrabPointer(false),
            mIsMouseRelative(false),
            mWrapPointerManually(false),
            mWarpCompensate(false),
            mWarpX(0),
            mWarpY(0),
            //
            mDisplayHelpMode(true),
            mDebugText(),
            mHelpDescription(""),
            mInteractiveElement2DList(),
            mViewingDirectionIndicator(nullptr),
            mWantRelative(false),
            mWantMouseVisible(true),
            mMouseManipulate(MM_NONE),
            mVRCameraNodeYaw(0),
            mVRCameraNodePitch(0),
            mVRCameraNodeTransZ(0),
            mVRCameraNodeTransX(0)
    {
        if( mIsStereo )
        {
            mTextureName[LEFT] = "VideoTextureLeft";
            mDatablockName[LEFT] = "VideoLeft";
            mTextureName[RIGHT] = "VideoTextureRight";
            mDatablockName[RIGHT] = "VideoRight";
        }
        else
        {
            mTextureName[0] = "VideoTextureMono";
            mDatablockName[0] = "VideoMono";
            // We don't use..
            mTextureName[1] = "VideoTextureNone";
            mDatablockName[1] = "VideoNone";
        }


    }

    void GameState::calcAlign(bool center)
    {
        StereoCameraConfig cameraConfig =
            mEsvr2->mVideoLoader->getStereoCameraConfig();
        if ( cameraConfig.leftToRight != 0.0f )
            mScale = mGraphicsSystem->mVrData.mLeftToRight.length() / cameraConfig.leftToRight;
        for( size_t eye = 0; eye < mEyeNum * 2; eye++ )
        {
            CameraConfig *cfg = cameraConfig.cfg [eye%2];
            Ogre::Real width = cfg->width;
            Ogre::Real height = cfg->height;
            Ogre::Real c_x = width/2;
            Ogre::Real c_y = height/2;
            Ogre::Real f_x, f_y;
            if (eye < mEyeNum)
            {
                f_x = cfg->K[0];
                f_y = cfg->K[4];
                if (!center)
                {
                    c_x = cfg->K[2];
                    c_y = cfg->K[5];
                }
                mCorrectProjPlaneDistance[eye] =
                        mEsvr2->mConfig->initialProjectionPlaneDistance;
            }
            else
            {
                f_x = cfg->P[0];
                f_y = cfg->P[5];
                if(!center)
                {
                    c_x = cfg->P[2];
                    c_y = cfg->P[6];
                }

                mCorrectProjPlaneDistance[eye] =
                        mEsvr2->mConfig->initialProjectionPlaneDistance;
//                    mGraphicsSystem->mVrData.mLeftToRight.length() * f_x /
//                    -cameraConfig.cfg[RIGHT]->P[3];
            }

            //in xy left is negativ
            mLeft[eye] = -c_x * mCorrectProjPlaneDistance[eye] / f_x;
            mRight[eye] = ( width -c_x  ) * mCorrectProjPlaneDistance[eye] / f_x;
            mTop[eye] = c_y * mCorrectProjPlaneDistance[eye] / f_y;
            mBottom[eye] = ( c_y - height  ) * mCorrectProjPlaneDistance[eye] / f_y;
            LOG << "mLeft: " << mLeft[eye] <<
                "mRight: " << mRight[eye] <<
                "mTop: " << mTop[eye] <<
                "mBottom: " << mBottom[eye] << LOGEND;
        }
    }

    //TODO: compiles but doesn't work
    void GameState::createProjectionRectangle2D()
    {
        mProjectionRectangle2D = mGraphicsSystem->mVRSceneManager
                ->createRectangle2D(Ogre::SCENE_DYNAMIC);
        mProjectionRectangle2D->setName("Rectangle2D");
        mProjectionRectangle2D->initialize(
            Ogre::BT_DEFAULT,
            Ogre::Rectangle2D::GeometryFlagQuad | Ogre::Rectangle2D::GeometryFlagNormals);

        // There's just simply no documentation
        mProjectionRectangle2D->setGeometry(
            Ogre::Vector2(-1.0f, -1.0f),
            Ogre::Vector2(2.0f,2.0f)
        );
        mProjectionRectangle2D->setDatablock(mDatablockName[LEFT]);
        mProjectionRectangle2D->setRenderQueueGroup( 212u );
        mGraphicsSystem->mVRSceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )
            ->attachObject(mProjectionRectangle2D);

        Ogre::MaterialManager &materialManager =
            Ogre::MaterialManager::getSingleton();

        Ogre::MaterialPtr material =
            materialManager.getByName( "SkyPostprocess", "Popular" );
        if( material )
        {
            LOG << "Could set material" << LOGEND;
            mProjectionRectangle2D->setMaterial(material);
        }
        else
        {
            LOG << "Error" << LOGEND;
            Ogre::ResourceManager::ResourceMapIterator it = materialManager.getResourceIterator();
            Ogre::ResourcePtr res;
            for(auto i = it.begin();
                i != it.end(); i++)
            {
                res = i->second;
                LOG << res->getName() << LOGEND;
            }
        }
    }

    void GameState::createVRProjectionPlanes()
    {
        bool alldata =
//             mVrData.mHeadToEye[LEFT] != Ogre::Matrix4::IDENTITY &&
            mCorrectProjPlaneDistance &&
            mLeft[LEFT] && mRight[LEFT] && mTop[LEFT] && mBottom[LEFT] &&
            ( !mIsStereo ||
                ( mLeft[RIGHT] && mRight[RIGHT] &&
                 mTop[RIGHT] && mBottom[RIGHT] )
            );
        if ( !alldata )
            return;

        mVRSceneNodeProjectionPlanesOrigin = mGraphicsSystem->mVRSceneManager
                ->getRootSceneNode()
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodeProjectionPlanesOrigin->setPosition(
                0, mEsvr2->mConfig->headHight, 0);

        mVRSceneNodesProjectionPlaneRaw = mVRSceneNodeProjectionPlanesOrigin
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRaw->setName("VR Node Projection Plane Raw");
        mVRSceneNodesProjectionPlaneRaw->setPosition(0, 0, -mCorrectProjPlaneDistance[DIST_RAW] );

        mVRSceneNodesProjectionPlaneRect = mVRSceneNodeProjectionPlanesOrigin
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRect->setName("VR Node Projection Plane Rect");
        mVRSceneNodesProjectionPlaneRect->setPosition(0, 0, -mCorrectProjPlaneDistance[DIST_UNDISTORT_RECTIFY] );
        Ogre::SceneNode *sceneNodesProjectionPlanes[2] =
                {mVRSceneNodesProjectionPlaneRaw, mVRSceneNodesProjectionPlaneRect};
        Ogre::Vector4 edge;
        //we need to create two planes for raw and recitified
        for( size_t eye = 0; eye < 2 * mEyeNum; eye++ )
        {
//             Ogre::Matrix4 eyeToHead =
//                 mVrData.mHeadToEye[eye %2]/*.inverse()*/;
//             Ogre::Vector4 camPos = eyeToHead *
//                 Ogre::Vector4( 0, 0, 0, 1.0 );
//             // Look back along -Z
//             //TODO: make focus dependend from scale is so we are focusing at about 10 cm.
//             // scale is about 10 so 10cm ~ 1m
// //             Ogre::Vector4 focusPoint = eyeToHead * Ogre::Vector4( 0.0, 0.0, -1.0, 1.0 );
//             mSceneNodeProjPlane[eye] = mSceneNodeCamera->createChildSceneNode(
//                 Ogre::SCENE_DYNAMIC );
//             mSceneNodeProjPlane[eye]->setPosition(camPos.xyz());
//
//             mSceneNodeProjPlane[eye]->lookAt(focusPoint.xyz(), Ogre::Node::TS_PARENT);

            mProjectionRectangle[eye] =
                    mGraphicsSystem->mVRSceneManager
                    ->createManualObject();

            mProjectionRectangle[eye]->begin(
                mDatablockName[eye%2], Ogre::OT_TRIANGLE_LIST);

            // Back
            edge = Ogre::Vector4( mLeft[eye], mTop[eye],
                               0, 1.0f );
            mProjectionRectangle[eye]->position( edge.xyz() );
            mProjectionRectangle[eye]->textureCoord(0 , 0);
            edge = Ogre::Vector4( mRight[eye], mTop[eye],
                               0, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 0);
            edge = Ogre::Vector4( mRight[eye], mBottom[eye],
                               0, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 1);
            edge = Ogre::Vector4( mLeft[eye], mBottom[eye],
                               0, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(0 , 1);
            mProjectionRectangle[eye]->quad(0, 1, 2, 3);
            mProjectionRectangle[eye]->quad(3, 2, 1, 0);

            mProjectionRectangle[eye]->end();

            sceneNodesProjectionPlanes[eye/2]->attachObject(mProjectionRectangle[eye]);
            mProjectionRectangle[eye]->setVisibilityFlags( 0x10 << eye );
        }
    }

    Ogre::ManualObject *GameState::createAxisIntern( Ogre::SceneManager *sceneManager )
    {
        Ogre::ManualObject *axis =
                sceneManager->createManualObject();
        Ogre::Real diam = 0.1f;
        Ogre::Real len = 1.0f;
        axis->begin("ColorRed", Ogre::OT_TRIANGLE_LIST);
        axis->position( 0.0f, diam, 0.0f );
        axis->position( 0.0f, -diam, 0.0f );
        axis->position( len, -diam, 0.0f );
        axis->position( len, diam, 0.0f );
        axis->quad(0, 1, 2, 3);
        axis->quad(3, 2, 1, 0);
        axis->position( 0.0f, 0.0f, diam );
        axis->position( 0.0f, 0.0f, -diam );
        axis->position( len, 0.0f, -diam );
        axis->position( len, 0.0f, diam );
        axis->quad(4, 5, 6, 7);
        axis->quad(7, 6, 5, 4);
        axis->end();
        axis->begin("ColorGreen", Ogre::OT_TRIANGLE_LIST);
        axis->position(  diam, 0.0f, 0.0f );
        axis->position( -diam, 0.0f, 0.0f );
        axis->position( -diam, len, 0.0f );
        axis->position(  diam, len, 0.0f );
        axis->quad(0, 1, 2, 3);
        axis->quad(3, 2, 1, 0);
        axis->position( 0.0f, 0.0f, diam );
        axis->position( 0.0f, 0.0f, -diam );
        axis->position( 0.0f, len, -diam );
        axis->position( 0.0f, len, diam );
        axis->quad(4, 5, 6, 7);
        axis->quad(7, 6, 5, 4);
        axis->end();
        axis->begin("ColorBlue", Ogre::OT_TRIANGLE_LIST);
        axis->position(  diam, 0.0f, 0.0f );
        axis->position( -diam, 0.0f, 0.0f );
        axis->position( -diam, 0.0f, len);
        axis->position( diam, 0.0f, len);
        axis->quad(0, 1, 2, 3);
        axis->quad(3, 2, 1, 0);
        axis->position( 0.0f, diam, 0.0f );
        axis->position( 0.0f, -diam, 0.0f );
        axis->position( 0.0f, -diam, len );
        axis->position( 0.0f, diam, len);
        axis->quad(4, 5, 6, 7);
        axis->quad(7, 6, 5, 4);
        axis->end();

        return axis;
    }

    void GameState::createLaparoscopeAxisCamera(void)
    {
        mAxis = createAxisIntern(mGraphicsSystem->mLaparoscopeSceneManager);
        mAxis->setName("AxisOrigin");
//        mAxis->setVisibilityFlags( 0x1 );
        mGraphicsSystem->mLaparoscopeSceneManager->getRootSceneNode(
            Ogre::SCENE_DYNAMIC )->attachObject(mAxis);
        mAxisCameras = createAxisIntern(mGraphicsSystem->mLaparoscopeSceneManager);
        mAxisCameras->setName("AxisCamera");
//        mAxisCameras->setVisibilityFlags( 0x1 << 1 );
        Ogre::SceneNode *camerasNode =
                mGraphicsSystem->mLaparoscopeSceneManager
                ->findSceneNodes ("Cameras Node")[0];
        camerasNode->attachObject(mAxisCameras);
    }

    void GameState::createVRAxisCamera(void)
    {
        mAxisVROrigin = createAxisIntern(mGraphicsSystem->mVRSceneManager);
        mAxisVROrigin->setName("Axis VR Origin");
//        mAxis->setVisibilityFlags( 0x1 );
        mGraphicsSystem->mVRSceneManager->getRootSceneNode(
            Ogre::SCENE_DYNAMIC )->attachObject(mAxisVROrigin);
        mAxisVRCameras = createAxisIntern(mGraphicsSystem->mVRSceneManager);
        mAxisVRCameras->setName("Axis VR Camera");
//        mAxisCameras->setVisibilityFlags( 0x1 << 1 );
        Ogre::DefaultSceneManager::SceneNodeList camerasNodes =
                mGraphicsSystem->mVRSceneManager
                ->findSceneNodes ("Cameras Node");
        mVRCamerasNode->attachObject(mAxisVRCameras);
    }

    void GameState::createTooltips( void )
    {
        mLaparoscopeSceneNodeTooltips = mGraphicsSystem->mLaparoscopeSceneManager->getRootSceneNode(
                Ogre::SCENE_DYNAMIC )->
                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeSceneNodeTooltips->setPosition( 0.0, 0.0, 0.0 );
        mTooltips = mGraphicsSystem->mLaparoscopeSceneManager->createBillboardSet();
        mTooltips->beginBillboards(1);
        Ogre::v1::Billboard* b = mTooltips->createBillboard(
            0.0, 0.0, 0.0);
        b->setDimensions(0.001 /** mScale*/, 0.001 /** mScale*/);
        b->setColour(Ogre::ColourValue::Red);
        mTooltips->endBillboards();
//        mTooltips->setVisibilityFlags( 0x1 );
        mLaparoscopeSceneNodeTooltips->attachObject( mTooltips );
    }


    void GameState::createMesh()
    {
        Ogre::Item *mCube = mGraphicsSystem->mVRSceneManager
                ->createItem(
                    "Cube_d.mesh",
                    Ogre::ResourceGroupManager::
                    AUTODETECT_RESOURCE_GROUP_NAME,
                    Ogre::SCENE_DYNAMIC );

        mCube->setVisibilityFlags( 0x1 << 2 );
        mVRSceneNodeMesh = mGraphicsSystem->mVRSceneManager
            ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
            ->createChildSceneNode( Ogre::SCENE_DYNAMIC );

        mVRSceneNodeMesh->setName("VR Node Mesh");
        mVRSceneNodeMesh->setPosition( 0, 0, -1.0 );

        mVRSceneNodeMesh->scale(0.25, 0.25, 0.25);
        mVRSceneNodeMesh->attachObject( mCube );
    }

#ifdef USE_POINTCLOUD
    void GameState::createPointCloud( void )
    {
        size_t numpoints = 100;
        Ogre::Real colorarray[numpoints*3];
        Ogre::Real pointlist[numpoints*3];
        for( size_t i = 0; i < numpoints; i++ )
        {
            size_t color_cnt = i*3;
            colorarray[color_cnt] = 1.0f;
            colorarray[color_cnt + 1] = 1.0f;
            colorarray[color_cnt + 2] = 1.0f;
            size_t point_cnt = i*3;
            pointlist[point_cnt] = (numpoints % 10) * 0.001 * mScale;
            pointlist[point_cnt + 1] = (numpoints / 10) * 0.001 * mScale;
            pointlist[point_cnt + 2] = -0.01 * mScale;
        }
        Ogre::String pcname = "PointCloud";
        Ogre::String pcEntName = "PointCloudEntity";
        mPointCloud = new PointCloud(
            pcEntName, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME, numpoints, pointlist, colorarray);

        Ogre::v1::Entity *pcEnt = mGraphicsSystem->mLaparoscopeSceneManager
                ->createEntity(mPointCloud->getMeshPtr());

        pcEnt->setMaterialName("Pointcloud");

        mLaparoscopeSceneNodePointCloud = mGraphicsSystem->mLaparoscopeSceneManager
            ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
            ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeSceneNodePointCloud->setPosition( 0, 0, -1.0 );
        mLaparoscopeSceneNodePointCloud->attachObject( pcEnt );
        pcEnt->setVisibilityFlags( 0x1 << 2 );
    }
#endif
    void GameState::createVROverlays(void)
    {
        addViewDirectionIndicator();
        InteractiveElementPtr ele = nullptr;
        ele = createInteractiveElement2D("GoBack",
                                   {Ogre::IdString(MENU_DISTANCE),
                                    Ogre::IdString(MENU_MOVE),
                                    Ogre::IdString(MENU_ADJUST_TO_HEAD),
                                    Ogre::IdString(MENU_CHANGE_DISTORTION),
                                    Ogre::IdString(MENU_CHANGE_CONTROLLER)});
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_GENERAL), nullptr));
//        ele = createInteractiveElement2D("MenuSlot9",
//                {Ogre::IdString(MENU_GENERAL)},
//                "Close Application");
//        ele->setTogglePressFunction(
//                boost::bind(&GraphicsSystem::quit, mGraphicsSystem));
        ele = createInteractiveElement2D("MenuSlot9",
                {Ogre::IdString(MENU_GENERAL)},
                "Close Menu");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        //ADJUST TO HEAD HIGHT
        mAdjustToHeadHightIE = std::make_shared<InteractiveElement>();
        mAdjustToHeadHightIE->setTogglePressFunction(
                boost::bind(&GameState::adjustToHeadHight, this));
        mAdjustToHeadHightIE->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        ele = createInteractiveElement2D("MenuSlot2",
                                   {Ogre::IdString(MENU_GENERAL)},
                                   "Adjust Projection-Plane to Head-Position");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_ADJUST_TO_HEAD), mAdjustToHeadHightIE));
        ele = createInteractiveElement2D("InfoBox",
                                   {Ogre::IdString(MENU_ADJUST_TO_HEAD)},
                                   "Hold the right foot-pedal to adjust\n"
                                   "the projection-plane to hight of head");
        //RESET PROJECTION PLANE DISTANCE
        ele = createInteractiveElement2D("MenuSlot3",
                   {Ogre::IdString(MENU_GENERAL)},
                   "Reset Projection-Plane Distance");
        ele->setTogglePressFunction(
                boost::bind(&GameState::resetProjectionPlaneDistance, this));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        //ADJUST PROJECTION PLANE DISTANCE
        mAdjustProjectionPlaneDistanceIE = std::make_shared<InteractiveElement>();
        mAdjustProjectionPlaneDistanceIE->setTogglePressFunction(
                boost::bind(&GameState::initAdjustProjectionPlaneDistance, this));
        mAdjustProjectionPlaneDistanceIE->setHoldFunction(
                boost::bind(&GameState::holdAdjustProjectionPlaneDistance, this, _1));
        mAdjustProjectionPlaneDistanceIE->setToggleReleaseFunction(
                boost::bind(&GameState::addSettingsEventLog, this, "adjustProjectionPlaneDistance"));
        ele = createInteractiveElement2D("InfoBox",
                                         {Ogre::IdString(MENU_DISTANCE),
                                          Ogre::IdString(MENU_SETUP_DISTANCE)},
                                         "Hold the right foot-pedal\n"
                                         "and turn your head\n"
                                         "to adjust the scaling.\n");
        ele = createInteractiveElement2D("MenuSlot4",
                                         {Ogre::IdString(MENU_GENERAL)},
                                         "Adjust Scaling");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_DISTANCE), mAdjustProjectionPlaneDistanceIE));
        // MOVE SCREEN
        mMoveIE = std::make_shared<InteractiveElement>();
        mMoveIE->setTogglePressFunction(
                boost::bind(&GameState::moveScreenInit, this));
        mMoveIE->setHoldFunction(
                boost::bind(&GameState::moveScreen, this, _1));
        mMoveIE->setToggleReleaseFunction(
                boost::bind(&GameState::addSettingsEventLog, this, "moveScreen"));
        ele = createInteractiveElement2D("MenuSlot5",
                   {Ogre::IdString(MENU_GENERAL)},
                   "MoveScreen");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_MOVE), mMoveIE));
        ele = createInteractiveElement2D("InfoBox",
                                         {Ogre::IdString(MENU_MOVE),
                                          Ogre::IdString(MENU_SETUP_MOVE)},
                                         "Hold the right foot-pedal\n"
                                         "and move your head\n"
                                         "to move the projection-plane\n"
                                         " to a comfortable position.");
        //CHANGE DISTORTION
        ele = createInteractiveElement2D("MenuSlot6",
                       {Ogre::IdString(MENU_GENERAL)},
                       "Change Distortion");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_CHANGE_DISTORTION), nullptr));
        ele = createInteractiveElement2D("MenuSlot5",
                    {Ogre::IdString(MENU_CHANGE_DISTORTION)},
                    "RawDist");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setDistortion, this, DIST_RAW));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        ele = createInteractiveElement2D("MenuSlot6",
                    {Ogre::IdString(MENU_CHANGE_DISTORTION)},
                    "UndistRectDist");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setDistortion, this, DIST_UNDISTORT_RECTIFY));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        mInfoBoxDistortionSelect = createInteractiveElement2D("InfoBox",
                                         {Ogre::IdString(MENU_CHANGE_DISTORTION)},
                                         "Current Distortion:");
        // CHANGE CONTROLLER
        ele = createInteractiveElement2D("MenuSlot7",
                       {Ogre::IdString(MENU_GENERAL)},
                       "Change Controller");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_CHANGE_CONTROLLER), nullptr));
        ele = createInteractiveElement2D("MenuSlot6",
                    {Ogre::IdString(MENU_CHANGE_CONTROLLER)},
                    "Opt0");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setController, this, CT_OPT0));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        ele = createInteractiveElement2D("MenuSlot7",
                    {Ogre::IdString(MENU_CHANGE_CONTROLLER)},
                    "Opt1");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setController, this, CT_OPT1));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        ele = createInteractiveElement2D("MenuSlot8",
                    {Ogre::IdString(MENU_CHANGE_CONTROLLER)},
                    "Opt2");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setController, this, CT_OPT2));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        ele = createInteractiveElement2D("MenuSlot9",
                    {Ogre::IdString(MENU_CHANGE_CONTROLLER)},
                    "None");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setController, this, CT_NONE));
        ele->setToggleReleaseFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(), nullptr));
        mInfoBoxControllerSelect = createInteractiveElement2D("InfoBox",
                                                        {Ogre::IdString(MENU_CHANGE_CONTROLLER)},
                                                        "Current Controller:");
        //DEBUG
        ele = createInteractiveElement2D("MenuSlot8",
                   {Ogre::IdString(MENU_GENERAL)},
                   "ShowDebug");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_DEBUG), nullptr));
        ele = createInteractiveElement2D("Debug",
                   {Ogre::IdString(MENU_DEBUG)});

        //VOID INTERACTIVE ELEMENT
        //An Interactive Element, which is not doing nothing
        mVoidIE = std::make_shared<InteractiveElement>();

        //MENU_SETUP
        ele = createInteractiveElement2D("AVictimBox",
                                         {Ogre::IdString(MENU_SETUP1)},
                                         "");
        ele = createInteractiveElement2D("SetupCenter",
                                         {Ogre::IdString(MENU_SETUP1)},
                                         "Make sure you see this text as sharp as possible:\n"
                                         "1. Check the headset sits tight on your head.\n"
                                         "2. Adjust the eye-distance of the headset\n"
                                         "using the dial on the lower right of the headset\n\n"
                                         "You can take off the headset at anytime you want.\n"
                                         "If you do so make sure\n"
                                         "that you are not pressing a foot-pedal.\n"
                                         "To continue, click the right foot-pedal.");
        mSetupStartIE = std::make_shared<InteractiveElement>();
        mSetupStartIE->setTogglePressFunction(
                boost::bind(&GameState::setupStart, this));

        //MENU_SETUP_CURSOR
        ele = createInteractiveElement2D("InfoBox",
                                         {Ogre::IdString(MENU_SETUP_CURSOR)},
                                        "Navigate the red dot\n"
                                        "using your head to the top-right \"Next\" button \n"
                                        "and click the right foot-pedal\n"
                                        "to select it.");
        ele = createInteractiveElement2D("AVictimBox",
                                         {Ogre::IdString(MENU_SETUP_CURSOR)},
                                         "Victim");
        ele = createInteractiveElement2D("SetupBtn1",
                                         {Ogre::IdString(MENU_SETUP_CURSOR)},
                                         "Skip Setup");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setupFinish, this));
        ele = createInteractiveElement2D("SetupBtn2",
                                         {Ogre::IdString(MENU_SETUP_CURSOR)},
                                         "Next\n"
                                         "Move Projection-Plane");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_MOVE), mMoveIE));
        //MENU_SETUP_MOVE
        //InfoBox of the Move Menu is also visible
        ele = createInteractiveElement2D("SetupBtn1",
                                         {Ogre::IdString(MENU_SETUP_MOVE)},
                                         "Back");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_CURSOR), mVoidIE));
        ele = createInteractiveElement2D("SetupBtn2",
                                         {Ogre::IdString(MENU_SETUP_MOVE)},
                                         "Next\n"
                                         "Scaling");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_DISTANCE), mAdjustProjectionPlaneDistanceIE));
        //MENU_SETUP_DISTANCE
        //InfoBox of the Adjust Projection Plane Distance Menu is also visible
        ele = createInteractiveElement2D("SetupBtn1",
                                         {Ogre::IdString(MENU_SETUP_DISTANCE)},
                                         "Back\n"
                                         "Move Projection-Plane");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_MOVE), mMoveIE));
        ele = createInteractiveElement2D("SetupBtn2",
                                         {Ogre::IdString(MENU_SETUP_DISTANCE)},
                                         "Next\n"
                                         "Explanations");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_EXPLANATION), mVoidIE));
        //MENU_SETUP_EXPLANATION
        ele = createInteractiveElement2D("SetupExplanation",
                                         {Ogre::IdString(MENU_SETUP_EXPLANATION)},
                                         "After you clicked \"Finish\",\n"
                                         "this UI will disappear.\n"
                                         "By hold the LEFT foot-pedal you activate\n"
                                         "the camera-controller.\n"
                                         "By clicking the RIGHT foot-pedal again\n"
                                         "a menu will show up.\n"
                                         "Please follow the instruction\n"
                                         "of the super-visor.");
        ele = createInteractiveElement2D("SetupBtn1",
                                         {Ogre::IdString(MENU_SETUP_EXPLANATION)},
                                         "Back\n"
                                         "Adjust Scaling");
        ele->setTogglePressFunction(
                boost::bind(&GameState::goToMenu, this, Ogre::IdString(MENU_SETUP_DISTANCE), mAdjustProjectionPlaneDistanceIE));
        ele = createInteractiveElement2D("SetupBtn2",
                                         {Ogre::IdString(MENU_SETUP_EXPLANATION)},
                                         "Finish");
        ele->setTogglePressFunction(
                boost::bind(&GameState::setupFinish, this));
    }

    void GameState::addViewDirectionIndicator()
    {
        Ogre::v1::OverlayManager &overlayManager =
                Ogre::v1::OverlayManager::getSingleton();
        Ogre::v1::Overlay *overlayViewDirectionIndicator =
                overlayManager.create( "ViewDirectionIndicator" );
        mViewingDirectionIndicator = static_cast<Ogre::v1::PanelOverlayElement*>(
                overlayManager.createOverlayElement("Panel", "ViewingDirecitonIndicator"));
//        mViewingDirectionIndicator->setColour(Ogre::ColourValue::Red);
        mViewingDirectionIndicator->setPosition( 0.5, 0.5 );
        mViewingDirectionIndicator->setHeight(0.01);
        mViewingDirectionIndicator->setWidth(0.01);
        mViewingDirectionIndicator->setMaterialName("ColorRed");
        overlayViewDirectionIndicator->add2D(mViewingDirectionIndicator);
        overlayViewDirectionIndicator->setRenderQueueGroup(254);
        overlayViewDirectionIndicator->show();

    }

    InteractiveElement2DPtr GameState::createInteractiveElement2D(
            Ogre::String defName,
            std::vector<Ogre::IdString> menus,
            Ogre::String text)
    {
        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = dynamic_cast<Ogre::HlmsUnlit*>(
                hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        InteractiveElement2DDefPtr defPtr =
                mGraphicsSystem->mInteractiveElementConfig.findByName(defName);
        if (defPtr)
        {
            InteractiveElement2DPtr element = std::make_shared<InteractiveElement2D>(
                    defPtr,
                    menus,
                    hlmsUnlit);
            addInteractiveElement2D(element);
            if (text == "")
                text = defPtr->text;
            element->setText(text);
            return element;
        }
        else
            return nullptr;
    }

    void GameState::addInteractiveElement2D(InteractiveElement2DPtr interactiveElement2D)
    {
        mInteractiveElement2DList.push_back( interactiveElement2D );
    }

    void GameState::createControllers()
    {
        if(mEsvr2->mLaparoscopeController)
        {
            mOpt0Controller =
                    std::make_shared<Opt0Controller>(
                    mEsvr2->mLaparoscopeController,
                    this,
                    mEsvr2->mConfig->ctlDelay,
                    mEsvr2->mConfig->ctlStepYaw,
                    mEsvr2->mConfig->ctlStepPitch,
                    mEsvr2->mConfig->ctlStepRoll,
                    mEsvr2->mConfig->ctlStepTransZ,
                    mEsvr2->mConfig->ctlOpt0ThresholdTransZ,
                    mEsvr2->mConfig->ctlOpt0ThresholdYawDeg,
                    mEsvr2->mConfig->ctlOpt0ThresholdPitchDeg,
                    mEsvr2->mConfig->ctlOpt0ThresholdRollDeg);
            mOpt1Controller =
                    std::make_shared<Opt1Controller>(
                    mEsvr2->mLaparoscopeController,
                    this,
                    mEsvr2->mConfig->ctlDelay,
                    mEsvr2->mConfig->ctlStepYaw,
                    mEsvr2->mConfig->ctlStepPitch,
                    mEsvr2->mConfig->ctlStepRoll,
                    mEsvr2->mConfig->ctlStepTransZ);
            mOpt2Controller =
                    std::make_shared<Opt2Controller>(
                    mEsvr2->mLaparoscopeController,
                    this,
                    mEsvr2->mConfig->ctlOpt2TransZFact,
                    mEsvr2->mConfig->ctlCameraTilt,
                    mEsvr2->mConfig->ctlFocusDistance);
        }
        else
        {
        LOG << "no laparoscope Controller defined, do not load Controller" << LOGEND;
        }
    }


    void GameState::loadDatablocks()
    {
        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = dynamic_cast<Ogre::HlmsUnlit*>(
                hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        InteractiveElementConfig config =
                mGraphicsSystem->mInteractiveElementConfig;

        for(auto it = config.colorDefList.begin();
                it != config.colorDefList.end(); it++)
        {
            ColorDef def = *it;
            Ogre::HlmsBlendblock blendBlock = Ogre::HlmsBlendblock();
            blendBlock.mBlockType = Ogre::BLOCK_BLEND;
            if (def.color[INDEX_ALPHA] < 1.0)
            {
                blendBlock.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
            }
            Ogre::HlmsUnlitDatablock* colourdatablock =
            dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                            hlmsUnlit->createDatablock(
                                    def.name,
                                    def.name,
                                    Ogre::HlmsMacroblock(),
                                    blendBlock,
                                    Ogre::HlmsParamVec() ) );
            colourdatablock->setUseColour(true);
            colourdatablock->setColour( Ogre::ColourValue(
                    def.color[INDEX_RED],def.color[INDEX_GREEN],
                    def.color[INDEX_GREEN],def.color[INDEX_ALPHA]));
        }

        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            Ogre::HlmsBlendblock blendBlock = Ogre::HlmsBlendblock();
            blendBlock.mBlockType = Ogre::BLOCK_BLEND;
            mVideoDatablock[eye] = dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                    hlmsUnlit->createDatablock(
                            mDatablockName[eye],
                            mDatablockName[eye],
                            Ogre::HlmsMacroblock(),
                            blendBlock,
                            Ogre::HlmsParamVec() ) );

            mVideoDatablock[eye]->setTexture( 0, mTextureName[eye] );
        }

        Ogre::HlmsBlendblock blendBlock = Ogre::HlmsBlendblock();
        blendBlock.mBlockType = Ogre::BLOCK_BLEND;
        blendBlock.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
        Ogre::String infoScreenDatablockName = "InfoScreenDatablock";
        mInfoScreenDatablock = dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                hlmsUnlit->createDatablock(
                        infoScreenDatablockName,
                        infoScreenDatablockName,
                        Ogre::HlmsMacroblock(),
                        blendBlock,
                        Ogre::HlmsParamVec() ) );

        // THis should work as well
//        mInfoScreenDatablock->setTexture(Ogre::TextureTypes::Type2D, mGraphicsSystem->mInfoScreenTexture);
        mInfoScreenDatablock->setTexture( 0, mGraphicsSystem->mInfoScreenTexture->getNameStr() );

        const Ogre::HlmsManager::HlmsDatablockMap datablocks = hlmsManager->getDatablocks();
        for(auto d = datablocks.begin(); d != datablocks.end(); d++ )
        {
            LOG << "HlmsDatablock: " << d->second->getName().getFriendlyText() << LOGEND;
        }
    }

    void GameState::createVRCamerasNodes()
    {
        mVRCamerasNode = mGraphicsSystem->mVRSceneManager
                ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
                ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mVRCamerasNode->setName( "VR Cameras Node" );
        //TODO if we do not have correct HMD position set a good initial position
        mVRCamerasNode->setPosition( 0,0,0 );

        mGraphicsSystem->mVRCullCamera->detachFromParent();
        mVRCamerasNode->attachObject(
                mGraphicsSystem->mVRCullCamera );

        for(size_t eye = 0; eye < mEyeNum; eye ++)
        {
            mGraphicsSystem->mVRCameras[eye]->detachFromParent();
            mVRCamerasNode->attachObject(
                    mGraphicsSystem->mVRCameras[eye] );
        }
        HmdConfig hmdConfig = mEsvr2->mConfig->hmdConfig;
        if (hmdConfig.valid())
        {
            Ogre::Real headHight = mEsvr2->mConfig->headHight;
            mVRCamerasNode->setPosition(Ogre::Vector3(0, headHight, 0));
        }
    }

    void GameState::createLaparoscopeCameraNodes()
    {
        mLaparoscopeCamerasNode = mGraphicsSystem->mLaparoscopeSceneManager
                ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
                ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mLaparoscopeCamerasNode->setName( "Lap Cameras Node" );
        mGraphicsSystem->mLaparoscopeCameras[LEFT]->detachFromParent();
        mLaparoscopeCamerasNode->attachObject(
                mGraphicsSystem->mLaparoscopeCameras[LEFT] );
        mGraphicsSystem->mLaparoscopeCameras[RIGHT]->detachFromParent();
        mLaparoscopeCamerasNode->attachObject(
                mGraphicsSystem->mLaparoscopeCameras[RIGHT] );

    }

    void GameState::createLaparoscopeScene()
    {
        //Follow the tip make the upper Hemisphere little more blue
        mGraphicsSystem->mLaparoscopeSceneManager->setAmbientLight(
                Ogre::ColourValue(0.6f, 0.8, 1.0),
                Ogre::ColourValue(1.0f, 0.8, 0.6),
                Ogre::Vector3(0.0f, 1.0f, 0.0f));

        createLaparoscopeCameraNodes();
//        createAxis();
//        createTooltips();
//         createPointCloud();

    }

    //-----------------------------------------------------------------------------------
    void GameState::createVRScene(void)
    {
        calcAlign(mEsvr2->mConfig->centerProjectionPlane);

        createVRCamerasNodes();
        createVRFloor();
//         createProjectionRectangle2D();
        createVRProjectionPlanes();
        createVRInfoScreen();
        createMesh();
//        createVRAxisCamera();

        Ogre::Light *light = mGraphicsSystem->mVRSceneManager->createLight();
        mVRSceneNodeLight = mGraphicsSystem->mVRSceneManager
                ->getRootSceneNode()->createChildSceneNode();
        mVRSceneNodeLight->setName("VR Node Light");
        mVRSceneNodeLight->attachObject( light );
        light->setPowerScale( Ogre::Math::PI ); //Since we don't do HDR, counter the PBS' division by PI
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        //Follow the tip make the upper Hemisphere little more blue
//        mGraphicsSystem->mVRSceneManager->setAmbientLight(
//                Ogre::ColourValue(0.6f, 0.8, 1.0),
//                Ogre::ColourValue(1.0f, 0.8, 0.6),
//                Ogre::Vector3(0.0f, 1.0f, 0.0f));


        mGraphicsSystem->mVRSceneManager->setVisibilityMask(0xFFFFFF30);

        createVROverlays();
        createControllers();

        if (mEsvr2->mConfig->startWithSetup)
        {
            mInfoScreenSceneNode->detachObject(mVRInfoScreen);
            mInfoScreenStaticSceneNode = mVRCamerasNode->
                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
            mInfoScreenStaticSceneNode->setName("InfoScreenStaticNode");
            mInfoScreenStaticSceneNode->setPosition( 0, 0, -mEsvr2->mConfig->infoScreenDistance);
            mInfoScreenStaticSceneNode->attachObject(mVRInfoScreen);
            mInfoScreenStaticSceneNode->setVisible(true);
            mVRSceneNodeProjectionPlanesOrigin->setVisible(false, true);
            goToMenu(Ogre::IdString(MENU_SETUP1), mSetupStartIE);
            //need to call this to show the first menu
            updateOverlayElements();
        }
        else
        {
            setController(mEsvr2->mConfig->controllerType);
        }
    }

    //-----------------------------------------------------------------------------------
    void GameState::generateDebugText(
            Ogre::uint64 microSecsSinceLast , Ogre::String &outText )
    {
        if( mDisplayHelpMode == 0 )
        {
            outText = mHelpDescription;
            outText += "\n\nPress F1 to toggle help";
            outText += "\n\nProtip: Ctrl+F1 will reload PBS shaders (for real time template editing).\n"
                       "Ctrl+F2 reloads Unlit shaders.\n"
                       "Ctrl+F3 reloads Compute shaders.\n"
                       "Note: If the modified templates produce invalid shader code, "
                       "crashes or exceptions can happen.\n";
            return;
        }

        const Ogre::FrameStats *frameStats = mGraphicsSystem->mRoot->getFrameStats();

        Ogre::Real fps = 1.0 / (microSecsSinceLast * 1000000);
        Ogre::String finalText;
        finalText.reserve( 128 );
        finalText += "Frame time:\t";
        finalText += Ogre::StringConverter::toString( microSecsSinceLast * 1000.0f );
        finalText += " ms\n";
        finalText += "Frame FPS:\t";
        finalText += Ogre::StringConverter::toString( fps );
        finalText += "\nAvg time:\t";
        finalText += Ogre::StringConverter::toString( frameStats->getAvgTime() );
        finalText += " ms\n";
        finalText += "Avg FPS:\t";
        finalText += Ogre::StringConverter::toString( 1000.0f / frameStats->getAvgTime() );
        finalText += "\n\nPress F1 to toggle help";

        outText.swap( finalText );

        InteractiveElement2DPtr interactiveDebug =
                findInteractiveElement2DByName("Debug");
        if (interactiveDebug)
            interactiveDebug->setText(finalText);
    }

    void GameState::updateLaparoscopePoseFromPoseState()
    {
        if ( mEsvr2->mPoseState && mEsvr2->mPoseState->validPose() )
        {
            Ogre::Quaternion prev_orientation =
                    mLaparoscopeCamerasNode->getOrientation();
            Ogre::Vector3 prev_position =
                    mLaparoscopeCamerasNode->getPosition();
            Ogre::Quaternion orientation =
                    RealArray4ToQuaternion(mEsvr2->mPoseState->getOrientation());
            orientation = orientation * Ogre::Quaternion(
                    Ogre::Degree(180), Ogre::Vector3::UNIT_Z);
            Ogre::Vector3 position =
                    RealArray3ToVector3(mEsvr2->mPoseState->getPosition());
            if ( !prev_orientation.orientationEquals(orientation)
                 || prev_position != position )
            {
//                 LOG << "update orientation and position" << mOvrCompositorListener->getFrameCnt();
//                 LOG << "Pos: " << position.x << " " << position.y << " " <<position.z;
//                 LOG << "Orientation: " << orientation.w << " "<< orientation.x << " " << orientation.y << " " <<orientation.z<<LOGEND;
                mLaparoscopeCamerasNode->setPosition( position );
                mLaparoscopeCamerasNode->setOrientation( orientation );
            }
        }
    }


    //-----------------------------------------------------------------------------------
    void GameState::setMouseRelative( bool relative )
    {
//        mWantMouseGrab = relative;
        mWantRelative = relative;
        updateMouseSettings();
    }
    //-----------------------------------------------------------------------------------
    void GameState::setMouseVisible( bool visible )
    {
        mWantMouseVisible = visible;
        updateMouseSettings();
    }
    void GameState::updateMouseSettings(void)
    {
        mGrabPointer = mMouseInWindow && mWindowHasFocus;
        SDL_SetWindowGrab( mGraphicsSystem->mSdlWindow, mGrabPointer ? SDL_TRUE : SDL_FALSE );

        SDL_ShowCursor( mWantMouseVisible || !mWindowHasFocus );

        bool relative = mWantRelative && mMouseInWindow && mWindowHasFocus;
        if( mIsMouseRelative == relative )
            return;

        mIsMouseRelative = relative;

        mWrapPointerManually = false;

        //Input driver doesn't support relative positioning. Do it manually.
        int success = SDL_SetRelativeMouseMode( relative ? SDL_TRUE : SDL_FALSE );
        if( !relative || (relative && success != 0) )
            mWrapPointerManually = true;

        //Remove all pending mouse events that were queued with the old settings.
        SDL_PumpEvents();
        SDL_FlushEvent( SDL_MOUSEMOTION );
    }

    void GameState::mouseMovedRelative(const SDL_Event &arg)
    {
        if(mMouseManipulate == MM_NONE)
            return;

        int width;
        int height;
        SDL_GetWindowSize( mGraphicsSystem->mSdlWindow, &width, &height );


        if (mMouseManipulate == MM_ORIENTATION)
        {
            mVRCameraNodeYaw   += -arg.motion.xrel /
                    static_cast<Ogre::Real>(width);
            mVRCameraNodePitch += -arg.motion.yrel /
                    static_cast<Ogre::Real>(height);
        }
        if (mMouseManipulate == MM_TRANSLATION)
        {
            mVRCameraNodeTransX   += -arg.motion.xrel /
                    static_cast<Ogre::Real>(width);
            mVRCameraNodeTransZ += -arg.motion.yrel /
                    static_cast<Ogre::Real>(height);
        }
        if (mMouseManipulate == MM_ROLL)
        {
            mVRCameraNodeRoll   += -arg.motion.xrel /
                    static_cast<Ogre::Real>(width);
        }
    }

    void GameState::mousePressed( const SDL_MouseButtonEvent &arg )
    {
        if ( arg.button == SDL_BUTTON_LEFT)
        {
//            setMouseRelative(true);
            mMouseManipulate = MM_ORIENTATION;
        }
        if ( arg.button == SDL_BUTTON_RIGHT)
        {
//            setMouseRelative(true);
            mMouseManipulate = MM_TRANSLATION;
        }
        if ( arg.button == SDL_BUTTON_MIDDLE)
        {
            mMouseManipulate = MM_ROLL;
        }
    }

    void GameState::mouseReleased( const SDL_MouseButtonEvent &arg )
    {
//        setMouseRelative(false);
        mMouseManipulate = MM_NONE;
    }
    //-----------------------------------------------------------------------------------
    void GameState::warpMouse( int x, int y )
    {
        SDL_WarpMouseInWindow( mGraphicsSystem->mSdlWindow, x, y );
        mWarpCompensate = true;
        mWarpX = x;
        mWarpY = y;
    }
    //-----------------------------------------------------------------------------------
    void GameState::wrapMousePointer( const SDL_MouseMotionEvent& evt )
    {
        //Don't wrap if we don't want relative movements, support
        //relative movements natively, or aren't grabbing anyways
        if( mIsMouseRelative || !mWrapPointerManually  )
            return;

        int width = 0;
        int height = 0;

        SDL_GetWindowSize( mGraphicsSystem->mSdlWindow, &width, &height );

        const int centerScreenX = width >> 1;
        const int centerScreenY = height >> 1;

        const int FUDGE_FACTOR_X = (width >> 2) - 1;
        const int FUDGE_FACTOR_Y = (height >> 2) - 1;

        //Warp the mouse if it's about to go outside the window
        if( evt.x <= centerScreenX - FUDGE_FACTOR_X || evt.x >= centerScreenX + FUDGE_FACTOR_X ||
            evt.y <= centerScreenY - FUDGE_FACTOR_Y || evt.y >= centerScreenY + FUDGE_FACTOR_Y )
        {
            warpMouse( centerScreenX, centerScreenY );
        }
    }

    void GameState::handleSdlEvent( const SDL_Event& evt )
    {
        switch( evt.type )
        {
            case SDL_MOUSEMOTION:
                // Ignore this if it happened due to a warp
//                if( !handleWarpMotion(evt.motion) ) {
                    // If in relative mode, don't trigger events unless window has focus
                    if (!mWantRelative || mWindowHasFocus)
                        mouseMovedRelative(evt);
                    // Try to keep the mouse inside the window
                    if (mWindowHasFocus)
                        wrapMousePointer(evt.motion);
//                }
                break;
            case SDL_MOUSEWHEEL:
//                mouseMoved( evt );
                break;
            case SDL_MOUSEBUTTONDOWN:
                mousePressed( evt.button );
                break;
            case SDL_MOUSEBUTTONUP:
                mouseReleased( evt.button );
                break;
            case SDL_KEYDOWN:
                if( !evt.key.repeat )
                {
                    bool handled = keyPressed(evt.key);
                }
                break;
            case SDL_KEYUP:
                if( !evt.key.repeat )
                {
                    bool handled = keyReleased( evt.key );
                }
                break;
        }
    }

    bool GameState::keyPressed( const SDL_KeyboardEvent &arg )
    {
        bool succ = false;
        if ( arg.keysym.scancode == SDL_SCANCODE_M ||
                (mCurrentController && !mCurrentController->isActiveOnPress() && arg.keysym.scancode == SDL_SCANCODE_N ))
        {
            //Make general UI visible
            if ( mUIStatus == UI_NONE )
            {
                mIsUIVisible = true;
                if (arg.keysym.scancode == SDL_SCANCODE_M)
                {
                    mUIStatus = UI_GENERAL;
                    mUIStatusStr = Ogre::IdString(MENU_GENERAL);
                }
                else
                {
                    mUIStatus = UI_CONTROLLER;
                    mUIStatusStr = Ogre::IdString(
                            mCurrentController->getControllerMenuId());
                }
                updateOverlayElements();
                succ = true;
            }
            else if ( (mUIStatus == UI_GENERAL || mUIStatus == UI_CONTROLLER)
                && mIsUIVisible )
            {
                if (mHoverUIElement)
                {
                    mUIActive = true;
                    mActiveUIElement = mHoverUIElement;
                    toggleUIPress();
                }
                else if (mBackgroundUIElement)
                {
                    mUIActive = true;
                    mActiveUIElement = mBackgroundUIElement;
                    toggleUIPress();
                }
                else
                {
                    goToMenu(Ogre::IdString(), nullptr);
                }
                updateOverlayElements();
                succ = true;
            }
        }
        else if (mCurrentController &&
                mCurrentController->isActiveOnPress() &&
                mUIStatus == UI_NONE &&
                arg.keysym.scancode == SDL_SCANCODE_N )
        {
            mUIActive = true;
            mActiveUIElement = mCurrentController;
            mUIStatus = UI_CONTROLLER_ACTIVE;
            toggleUIPress();
            succ = true;
        }
        return succ;
    }

    bool GameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
        if( arg.keysym.scancode == SDL_SCANCODE_ESCAPE )
        {
            mGraphicsSystem->quit();
            return true;
        }
        bool succ = false;
        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::uint32 flipMask = 0x0;
        Ogre::uint32 setMask = 0x0;
        Ogre::uint32 unsetMask = 0x0;

        if( arg.keysym.scancode == SDL_SCANCODE_Y )
        {
            setDistortion(DIST_RAW);
            succ = true;
        }
        if( arg.keysym.scancode == SDL_SCANCODE_X )
        {
            setDistortion(DIST_UNDISTORT_RECTIFY);
            succ = true;
        }
//        if( arg.keysym.scancode == SDL_SCANCODE_A )
//        {
//            Ogre::Real zoom = mGraphicsSystem->getZoom();
//            zoom += arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL) ? 0.1 : -0.1;
//            mGraphicsSystem->setZoom( zoom );
//            succ = true;
//        }
        if( arg.keysym.scancode == SDL_SCANCODE_1 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask();
            if (visibilityMask & 0xF0)
                unsetMask = 0xF0;
            else
            {
                Distortion dist = mEsvr2->mVideoLoader->getDistortion();
                if (dist == DIST_UNDISTORT_RECTIFY)
                {
                    setMask = 0x40 | 0x80;
                    unsetMask = 0x10 | 0x20;
                }
                else
                {
                    setMask = 0x10 | 0x20;
                    unsetMask = 0x40 | 0x80;
                }
            }
            succ = true;
        }
        //strg + 2 axis
        if( arg.keysym.scancode == SDL_SCANCODE_2 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1;
            succ = true;
        }
        //strg + 4 mesh
        if( arg.keysym.scancode == SDL_SCANCODE_3 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1 << 1;
            succ = true;
        }
        //strg + 3 point cloud
        if( arg.keysym.scancode == SDL_SCANCODE_4 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1 << 2;
            succ = true;
        }

        //TODO: It's too complicated for just flipping certain bits
        Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask();
        visibilityMask &= ~flipMask;
        visibilityMask |= ~sceneManager->getVisibilityMask() & flipMask;
        visibilityMask |= setMask;
        visibilityMask &= ~unsetMask;
        sceneManager->setVisibilityMask( visibilityMask );

        // stop Video
        if( arg.keysym.scancode == SDL_SCANCODE_SPACE )
        {
            mGraphicsSystem->toggleShowVideo();
            succ = true;
        }

        if( (arg.keysym.scancode == SDL_SCANCODE_M && mUIStatus == UI_GENERAL) ||
                (arg.keysym.scancode == SDL_SCANCODE_N &&
                        (mUIStatus == UI_CONTROLLER || mUIStatus == UI_CONTROLLER_ACTIVE )))
        {
            toggleUIRelease();
            mActiveUIElement = nullptr;
            mUIActive = false;
            if (mUIStatus == UI_CONTROLLER_ACTIVE)
            {
                mBackgroundUIElement = nullptr;
                mUIStatus = UI_NONE;
            }
            updateOverlayElements();
            succ = true;
        }
        return succ;
    }

#ifdef USE_FOOTPEDAL
    void GameState::handleFootPedalEvent( const FootPedalEvent& evt )
    {
        if ( evt == FPE_HOVER_PRESS )
        {
            mIsUIVisible = true;
            updateOverlayElements();
        }
        if( evt == FPE_ACTIVE_PRESS && mIsUIVisible )
        {
            toggleUIPress();
            mUIActive = true;
            mActiveUIElement = mHoverUIElement;
            updateOverlayElements();
        }
        if( evt == FPE_HOVER_RELEASE )
        {
            mHoverUIElement = nullptr;
            mIsUIVisible = false;
            updateOverlayElements();
        }
        if(  evt == FPE_ACTIVE_RELEASE )
        {
            mActiveUIElement = nullptr;
            mUIActive = false;
            updateOverlayElements();
        }
    }
#endif

    void GameState::updateOverlayElements()
    {
        bool hideOther = mHoverUIElement ?
                mHoverUIElement->isHideOtherOnActive() : false;
        for (auto it = mInteractiveElement2DList.begin();
                it != mInteractiveElement2DList.end();
                it++) {
            InteractiveElement2DPtr elem = *it;
            if ((mIsUIVisible || mUIActive) &&
                elem->isVisibleByMenu(mUIStatusStr)) {
                if (elem == mHoverUIElement && mIsUIVisible && !mUIActive) {
                    elem->setUIState(UIS_HOVER);
                } else if (elem == mActiveUIElement && mUIActive && elem->isVisibleOnActive()) {
                    elem->setUIState(UIS_ACTIVATE);
                } else if (elem == mActiveUIElement && mUIActive && !elem->isVisibleOnActive()) {
                    elem->setUIState(UIS_NONE);
                } else if (elem != mHoverUIElement && mUIActive && hideOther) {
                    elem->setUIState(UIS_NONE);
                } else {
                    elem->setUIState(UIS_VISIBLE);
                }
            } else {
                elem->setUIState(UIS_NONE);
            }
        }
    }

    void GameState::toggleUIPress()
    {
        if (mActiveUIElement)
            mActiveUIElement->togglePress();
    }

    void GameState::holdUI(Ogre::uint64 currentTimeMs)
    {
        if (mActiveUIElement)
            mActiveUIElement->hold(currentTimeMs);
    }

    void GameState::toggleUIRelease()
    {
        if (mActiveUIElement)
            mActiveUIElement->toggleRelease();
    }

    bool GameState::setDistortion(Distortion dist)
    {
        mEsvr2->mVideoLoader->setDistortion(dist);
        Ogre::uint32 setMask = 0x0;
        Ogre::uint32 unsetMask = 0x0;
        Ogre::String txt = "";
        if (dist == DIST_RAW)
        {
            setMask = 0x10 | 0x20;
            unsetMask = 0x40 | 0x80;
            txt = "Current: Raw";
        }
        else if (dist == DIST_UNDISTORT_RECTIFY)
        {
            setMask = 0x40 | 0x80;
            unsetMask = 0x10 | 0x20;
            txt = "Current: Undist Rect";
        }
        if (mInfoBoxDistortionSelect)
            mInfoBoxDistortionSelect->setText(txt);
        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask( );
        visibilityMask |= setMask;
        visibilityMask &= ~unsetMask;
        sceneManager->setVisibilityMask( visibilityMask );
        return true;
    }

    void GameState::updateVRCamerasNode(void)
    {
        mVRCamerasNode->_getFullTransformUpdated();
        mVRCamerasNode->yaw(Ogre::Radian(mVRCameraNodeYaw));
        mVRCamerasNode->pitch(Ogre::Radian(mVRCameraNodePitch));
        mVRCamerasNode->roll(Ogre::Radian(mVRCameraNodeRoll));
        Ogre::Vector3 pos = mVRCamerasNode->getPosition();
        Ogre::Vector3 newPos = Ogre::Vector3(
                pos.x + mVRCameraNodeTransX,
                pos.y,
                pos.z + mVRCameraNodeTransZ);
        mVRCamerasNode->setPosition(newPos);
        mVRCameraNodeYaw = 0;
        mVRCameraNodePitch = 0;
        mVRCameraNodeTransX = 0;
        mVRCameraNodeTransZ = 0;
        mVRCameraNodeRoll = 0;
    }

    //-----------------------------------------------------------------------------------
    void GameState::update( Ogre::uint64 msSinceLast )
    {
        updateVRCamerasNode();
        readHeadGestures();
        if (mCurrentController )
            mCurrentController->headPoseUpdated();
        //update Pointcloud ?
        if( mDisplayHelpMode && mGraphicsSystem->mOverlaySystem && mUIStatusStr == Ogre::IdString(MENU_DEBUG) )
        {
            if (mDebugText != "")
            {
                InteractiveElement2DPtr interactiveDebug =
                        findInteractiveElement2DByName("Debug");
                if (interactiveDebug)
                    interactiveDebug->setText(mDebugText);
            }
            else
            {
                //Show FPS
                Ogre::String finalText;
                generateDebugText(msSinceLast, finalText );
                InteractiveElement2DPtr interactiveDebug =
                        findInteractiveElement2DByName("Debug");
                if (interactiveDebug)
                    interactiveDebug->setText(finalText);
            }
        }
        if (mUIActive)
        {
            holdUI(mGraphicsSystem->mLastStartTime);
        }
        else if (mIsUIVisible)
        {
            InteractiveElement2DPtr elem = nullptr;
            for (auto it = mInteractiveElement2DList.begin();
                it != mInteractiveElement2DList.end(); it++)
            {
                if((*it)->isVisibleByMenu(mUIStatusStr) &&
                    (*it)->isUVinside(mInfoScreenUV) &&
                    (*it)->isActivatable())
                {
                    elem = *it;
                    break;
                }
            }
            //change Hoverelement
            if (mHoverUIElement != elem)
            {
                if (mHoverUIElement)
                    mHoverUIElement->setUIState(UIS_VISIBLE);
                if (elem)
                    elem->setUIState(UIS_HOVER);
                mHoverUIElement = elem;
            }
        }
        else
        {
            if (mHoverUIElement)
            {
                mHoverUIElement->setUIState(UIS_VISIBLE);
                mHoverUIElement = nullptr;
            }
        }
    }

    InteractiveElement2DPtr GameState::findInteractiveElement2DByName(
            Ogre::String id)
    {
        auto it = find_if(
                mInteractiveElement2DList.begin(),
                mInteractiveElement2DList.end(),
                [&id](const InteractiveElement2DPtr& obj)
                    {return obj->getId() == id;});
        return it != mInteractiveElement2DList.end() ? *it : nullptr;
    }

    InteractiveElement2DPtr GameState::findInteractiveElement2DByUV(
            Ogre::Vector2 uv)
    {
        auto it = find_if(
                mInteractiveElement2DList.begin(),
                mInteractiveElement2DList.end(),
                [&uv](const InteractiveElement2DPtr& obj)
                {return obj->isUVinside(uv);});
        return it != mInteractiveElement2DList.end() ? *it : nullptr;
    }

    void GameState::createVRInfoScreen(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        mVRInfoScreen = sceneManager->createManualObject();

        mVRInfoScreen->begin(
                *(mInfoScreenDatablock->getNameStr()),
                Ogre::OT_TRIANGLE_LIST);

        Ogre::Real x = mGraphicsSystem->mInteractiveElementConfig.width/2.0;
        Ogre::Real y = mGraphicsSystem->mInteractiveElementConfig.height/2.0;
        Ogre::Vector3 edge;
        edge = Ogre::Vector3(-x, y,0);
        mVRInfoScreen->position( edge );
        mVRInfoScreen->textureCoord(0 , 0);
        edge = Ogre::Vector3( x, y, 0 );
        mVRInfoScreen->position(edge);
        mVRInfoScreen->textureCoord(1 , 0);
        edge = Ogre::Vector3( x, -y, 0 );
        mVRInfoScreen->position(edge);
        mVRInfoScreen->textureCoord(1 , 1);
        edge = Ogre::Vector3( -x, -y, 0);
        mVRInfoScreen->position(edge);
        mVRInfoScreen->textureCoord(0 , 1);
        mVRInfoScreen->quad(3, 2, 1, 0);

        mVRInfoScreen->end();
        mInfoScreenSceneNode = mVRSceneNodeProjectionPlanesOrigin->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mInfoScreenSceneNode->setName("InfoScreenNode");
        mInfoScreenSceneNode->attachObject( mVRInfoScreen );
        //for debugging
//        mInfoScreenSceneNode->attachObject( createAxisIntern(sceneManager));
        Ogre::Real dist = mCorrectProjPlaneDistance[DIST_RAW];
        mInfoScreenSceneNode->setPosition( 0, 0, -mEsvr2->mConfig->infoScreenDistance);
    }

    void GameState::createVRFloor()
    {
        Ogre::v1::MeshPtr planeMeshV1 = Ogre::v1::MeshManager::getSingleton().createPlane(
                "Plane v1",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
               Ogre::Plane( Ogre::Vector3::UNIT_Y, 0.0f ), 1.0f, 1.0f,
               1, 1, true, 1, 0.1f, 0.1f, Ogre::Vector3::UNIT_Z,
               Ogre::v1::HardwareBuffer::HBU_STATIC,
               Ogre::v1::HardwareBuffer::HBU_STATIC );

        Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createByImportingV1(
                "Plane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                planeMeshV1.get(), true, true, true );

        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::Item *item = sceneManager->createItem( planeMesh, Ogre::SCENE_DYNAMIC );

//        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        item->setDatablock( "Marble" );
        mFloorSceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mFloorSceneNode->setPosition( 0, 0, 0 );
        mFloorSceneNode->attachObject( item );

        //Change the addressing mode of the roughness map to wrap via code.
        //Detail maps default to wrap, but the rest to clamp.
//            assert( dynamic_cast<Ogre::HlmsPbsDatablock*>( item->getSubItem(0)->getDatablock() ) );
//            Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
//                    item->getSubItem(0)->getDatablock() );
//            //Make a hard copy of the sampler block
//            Ogre::HlmsSamplerblock samplerblock( *datablock->getSamplerblock( Ogre::PBSM_ROUGHNESS ) );
//            samplerblock.mU = Ogre::TAM_WRAP;
//            samplerblock.mV = Ogre::TAM_WRAP;
//            samplerblock.mW = Ogre::TAM_WRAP;
//            //Set the new samplerblock. The Hlms system will
//            //automatically create the API block if necessary
//            datablock->setSamplerblock( Ogre::PBSM_ROUGHNESS, samplerblock );
    }

    void GameState::readHeadGestures()
    {
        Ogre::Vector3 origin = mVRCamerasNode->getPosition();
        Ogre::Vector3 direction = mVRCamerasNode->getOrientation() * (- Ogre::Vector3::UNIT_Z);
        Ogre::Ray viewingDirection = Ogre::Ray(origin, direction);

        Ogre::Plane infoScreenPlane(
                mInfoScreenSceneNode->convertLocalToWorldDirectionUpdated(Ogre::Vector3::UNIT_Z, true),
                mInfoScreenSceneNode->convertLocalToWorldPositionUpdated(Ogre::Vector3::ZERO));
        std::pair<bool, Ogre::Real> pairIntersections =
                Ogre::Math::intersects(viewingDirection, infoScreenPlane);
        Ogre::Vector3 intersect = viewingDirection.getPoint(pairIntersections.second);
        Ogre::Vector3 intersect_loc = mInfoScreenSceneNode->convertWorldToLocalPosition(intersect);
        Ogre::Vector2 uv;
        Ogre::Real infoScreenDimX = mGraphicsSystem->mInteractiveElementConfig.width/2.0;
        Ogre::Real infoScreenDimY = mGraphicsSystem->mInteractiveElementConfig.height/2.0;
        uv.x = (intersect_loc.x + infoScreenDimX) / (2 * infoScreenDimX);
        uv.y = (intersect_loc.y - infoScreenDimY) / (-2 * infoScreenDimY);
        bool intersectsInfoScreen = 0.0f < uv.x && uv.x < 1.0f &&  0 < uv.y && uv.y < 1.0f;
        if (mViewingDirectionIndicator)
        {
            if (intersectsInfoScreen &&
                mIsUIVisible && !mUIActive &&
                mShowViewingDirectionIndicator)
            {
                mInfoScreenUV = uv;
                mViewingDirectionIndicator->setPosition(uv.x, uv.y);
                mViewingDirectionIndicator->show();
            }
            else
            {
                mViewingDirectionIndicator->hide();
            }
        }
    }

    void GameState::adjustToHeadHight()
    {
        Ogre::Vector3 headPose = mVRCamerasNode->getPosition();
        mVRSceneNodeProjectionPlanesOrigin->setPosition(
                mVRCamerasNode->getPosition());
        mFloorSceneNode->setPosition(
                Ogre::Vector3(headPose.x, 0, headPose.z));
        addSettingsEventLog("adjustToHeadHight");
    }

    void GameState::moveScreenInit()
    {
        mVRSceneNodeProjectionPlanesOrigin->setPosition(
                mVRCamerasNode->getPosition());
    }

    void GameState::moveScreen(Ogre::uint64 time)
    {
        //TODO: guard mVRSceneNodeProjectionPlanesOrigin and
        // mVRCamerasNode are at least in proxemity like 0.1m~
        Ogre::Quaternion headOrientation = mVRCamerasNode->getOrientation();
        Ogre::Vector3 xAxisTrans = headOrientation.xAxis();
        if (xAxisTrans == Ogre::Vector3::UNIT_Y &&
                xAxisTrans == -Ogre::Vector3::UNIT_Y)
            return;
        Ogre::Vector3 xAxisNew(xAxisTrans.x,0,xAxisTrans.z);
        xAxisNew.normalise();
        Ogre::Quaternion trans = xAxisTrans.getRotationTo(xAxisNew);
        mVRSceneNodeProjectionPlanesOrigin->setOrientation(
                trans * headOrientation);
    }

    void GameState::resetProjectionPlaneDistance()
    {
        mVRSceneNodesProjectionPlaneRaw->setPosition(
                0, 0, -mCorrectProjPlaneDistance[DIST_RAW]);
        mVRSceneNodesProjectionPlaneRect->setPosition(
                0, 0, -mCorrectProjPlaneDistance[DIST_UNDISTORT_RECTIFY] );
        addSettingsEventLog("resetProjectionPlaneDistance");
    }

    void GameState::goToMenu(Ogre::IdString menu, InteractiveElementPtr backgroundIE)
    {
        if(menu == Ogre::IdString())
        {
            mIsUIVisible = false;
            mUIStatus = UI_NONE;
        }
        else
        {
            mIsUIVisible = true;
            mUIStatus = UI_GENERAL;
        }
        mUIStatusStr = menu;
        mBackgroundUIElement = backgroundIE;
        mHoverUIElement = nullptr;
    }

    void GameState::initAdjustProjectionPlaneDistance()
    {
        mAdjustProjectionPlaneInitialPitch =
                getHeadOrientation().getYaw().valueRadians();
        mAdjustProjectionPlaneRawInitialDistance =
                mVRSceneNodesProjectionPlaneRaw->getPosition().z;
        mAdjustProjectionPlaneRectInitialDistance =
                   mVRSceneNodesProjectionPlaneRect->getPosition().z;
    }

    void GameState::holdAdjustProjectionPlaneDistance(Ogre::uint64 time)
    {
        Ogre::Real minDistance = -0.4;
        Ogre::Real maxDistance = -10;
        Ogre::Real increment = mAdjustProjectionPlaneInitialPitch -
                getHeadOrientation().getYaw().valueRadians();
        Ogre::Real newProjectionPlaneDistanceRaw =
                mAdjustProjectionPlaneRawInitialDistance +
                (mAdjustProjectionPlaneFact * increment);
        newProjectionPlaneDistanceRaw = std::min(newProjectionPlaneDistanceRaw, minDistance);
        newProjectionPlaneDistanceRaw = std::max(newProjectionPlaneDistanceRaw, maxDistance);
        mVRSceneNodesProjectionPlaneRaw->setPosition(
                0, 0, newProjectionPlaneDistanceRaw);
        Ogre::Real newProjectionPlaneDistanceRect =
                mAdjustProjectionPlaneRectInitialDistance +
                (mAdjustProjectionPlaneFact * increment);
        newProjectionPlaneDistanceRect = std::min(newProjectionPlaneDistanceRect, minDistance);
        newProjectionPlaneDistanceRect = std::max(newProjectionPlaneDistanceRect, maxDistance);
        mVRSceneNodesProjectionPlaneRect->setPosition(
                0, 0, newProjectionPlaneDistanceRect );
    }

    Ogre::Quaternion GameState::getHeadOrientation()
    {
        return mVRCamerasNode->getOrientation();
    }

    Ogre::Quaternion GameState::getProjectionPlanesOrientation()
    {
        return mVRSceneNodeProjectionPlanesOrigin->getOrientation();
    }

    Ogre::Vector3 GameState::getHeadPosition()
    {
        return mVRCamerasNode->getPosition();
    }

    bool GameState::isHeadPositionCentered()
    {
        Ogre::Vector3 diff =
            mVRSceneNodeProjectionPlanesOrigin->getPosition()
            - mVRCamerasNode->getPosition();
        return diff.length() < mEsvr2->mConfig->centerEpsilon;
    }

    void GameState::setDebugText(Ogre::String debugText)
    {
        mDebugText = debugText;
    }

    void GameState::setController(ControllerType ct)
    {
        Ogre::String txt = "";
        switch (ct)
        {
            case CT_OPT0:
                mCurrentController = mOpt0Controller;
                txt = "Current: Opt0";
                break;
            case CT_OPT1:
                mCurrentController = mOpt1Controller;
                txt = "Current: Opt1";
                break;
            case CT_OPT2:
                mCurrentController = mOpt2Controller;
                txt = "Current: Opt2";
                break;
            default:
                mCurrentController = nullptr;
                txt = "Current: None";
        }
        if (mInfoBoxControllerSelect)
            mInfoBoxControllerSelect->setText(txt);
    }

    void GameState::addSettingsEventLog(Ogre::String eventStr)
    {
        SettingsEventLog log;
        log.time = mGraphicsSystem->mLastStartTime;
        log.event = eventStr;
        log.projectionPlaneDistanceRaw =
                mVRSceneNodesProjectionPlaneRaw->getPosition().z;
        log.projectionPlaneDistanceRect =
                mVRSceneNodesProjectionPlaneRect->getPosition().z;
        log.projectionPlaneOrientation =
                mVRSceneNodeProjectionPlanesOrigin->getOrientation();
        log.headPosition =
                mVRSceneNodeProjectionPlanesOrigin->getPosition();
        log.eyeDist =
                mGraphicsSystem->mVRCameras[RIGHT]->getPosition().x;
        log.distortion = mEsvr2->mVideoLoader->getDistortion();
        mSettingsEventLogs.push_back(log);
    }

    void GameState::setupStart()
    {
        adjustToHeadHight();
        moveScreen(0);
        //detach from parent
        mInfoScreenStaticSceneNode->detachObject(mVRInfoScreen);
        mInfoScreenSceneNode->attachObject(mVRInfoScreen);
        mVRSceneNodeProjectionPlanesOrigin->setVisible(true, true);
        mShowViewingDirectionIndicator = true;
        //TODO: if I callthis in the target Menu
        // in the alphanumeric lowest Interactive Element the Text disappears
        goToMenu(Ogre::IdString(MENU_SETUP_CURSOR), mVoidIE);
    }
    void GameState::setupFinish()
    {
        setController(mEsvr2->mConfig->controllerType);
        goToMenu(Ogre::IdString(), nullptr);
    }
}