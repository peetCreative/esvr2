#include "Esvr2GameState.h"

#include "Esvr2.h"
#include "Esvr2PointCloud.h"
#include "Esvr2InteractiveElement2D.h"
#include "Esvr2Helper.h"
#include "Esvr2Controller.h"

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
#include "Overlay/OgreOverlayContainer.h"
#include "Overlay/OgreTextAreaOverlayElement.h"
#include "Overlay/OgreBorderPanelOverlayElement.h"

#include <boost/bind/bind.hpp>
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
            mPointCloud( nullptr ),
            mVRSceneNodeLight( nullptr ),
            mVRSceneNodeMesh( nullptr ),
            mVRSceneNodeProjectionPlanesOrigin(nullptr),
            mVRSceneNodesProjectionPlaneRaw(nullptr),
            mVRSceneNodesProjectionPlaneRect(nullptr),
            mLaparoscopeSceneNodePointCloud( nullptr ),
            mLaparoscopeSceneNodeTooltips( nullptr ),
            mInfoScreenSceneNode(nullptr),
            mIntersectsInfoScreen(false),
//            mUIStatus(UIS_NONE),
            mIsUIVisible(false),
            mHoverUIElement(nullptr),
            mUIActive(false),
            mActiveUIElement(nullptr),
            mInfoScreenUV( 0, 0 ),
            mIsStereo( esvr2->mConfig->isStereo ),
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 ),
            mProjPlaneDistance{ 0, 0, 0, 0 },
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
        float projPlaneDistance = 1.0f;
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
                mProjPlaneDistance[eye] = projPlaneDistance;
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

                mProjPlaneDistance[eye] =
                    mGraphicsSystem->mVrData.mLeftToRight.length() * f_x /
                    -cameraConfig.cfg[RIGHT]->P[3];
            }

            //in xy left is negativ
            mLeft[eye] = -c_x * mProjPlaneDistance[eye] / f_x;
            mRight[eye] = ( width -c_x  ) * mProjPlaneDistance[eye] / f_x;
            mTop[eye] = c_y * mProjPlaneDistance[eye] / f_y;
            mBottom[eye] = ( c_y - height  ) * mProjPlaneDistance[eye] / f_y;
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
            mProjPlaneDistance &&
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
        mVRSceneNodeProjectionPlanesOrigin->pitch(
                Ogre::Radian(Ogre::Degree(-30)), Ogre::Node::TS_LOCAL);

        mVRSceneNodesProjectionPlaneRaw = mVRSceneNodeProjectionPlanesOrigin
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRaw->setName("VR Node Projection Plane Raw");
        mVRSceneNodesProjectionPlaneRaw->translate(0, 0, -mProjPlaneDistance[DIST_RAW] );

        mVRSceneNodesProjectionPlaneRect = mVRSceneNodeProjectionPlanesOrigin
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRect->setName("VR Node Projection Plane Rect");
        Ogre::SceneNode *sceneNodesProjectionPlanes[2] =
                {mVRSceneNodesProjectionPlaneRaw, mVRSceneNodesProjectionPlaneRect};
        mVRSceneNodesProjectionPlaneRaw->translate(0, 0, -mProjPlaneDistance[DIST_UNDISTORT_RECTIFY] );
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

    void GameState::createVROverlays(void)
    {
        addViewDirectionIndicator();

        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = dynamic_cast<Ogre::HlmsUnlit*>(
                hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        InteractiveElementConfig config =
                mGraphicsSystem->mInteractiveElementConfig;

        InteractiveElement2DDefPtr debugElementDefPtr =
            config.findByName("Debug");
        if (debugElementDefPtr)
        {
            InteractiveElement2DPtr element = std::make_shared<InteractiveElement2D>(
                    debugElementDefPtr,
                    (boost::function<void ()>) 0,
                    (boost::function<void (Ogre::uint64)>) 0,
                    hlmsUnlit);
            addInteractiveElement2D(element);
        }

        InteractiveElement2DDefPtr closeDefPtr =
            mGraphicsSystem->mInteractiveElementConfig.findByName("Close");
        if (closeDefPtr)
        {
            InteractiveElement2DPtr element = std::make_shared<InteractiveElement2D>(
                    closeDefPtr,
                    boost::bind(&GraphicsSystem::quit, mGraphicsSystem),
                    (boost::function<void(Ogre::uint64)>) 0,
                    hlmsUnlit);
            addInteractiveElement2D(element);
        }

        InteractiveElement2DDefPtr RawDistDefPtr =
            mGraphicsSystem->mInteractiveElementConfig.findByName("RawDist");
        if (RawDistDefPtr)
        {
            InteractiveElement2DPtr element = std::make_shared<InteractiveElement2D>(
                    RawDistDefPtr,
                    boost::bind(&GameState::setDistortion, this, DIST_RAW),
                    (boost::function<void(Ogre::uint64)>) 0,
                    hlmsUnlit);
            addInteractiveElement2D(element);
        }

        InteractiveElement2DDefPtr UndistRectDistDefPtr =
            mGraphicsSystem->mInteractiveElementConfig.findByName("UndistRectDist");
        if (UndistRectDistDefPtr)
        {
            InteractiveElement2DPtr element = std::make_shared<InteractiveElement2D>(
                    UndistRectDistDefPtr,
                    boost::bind(&GameState::setDistortion, this, DIST_UNDISTORT_RECTIFY),
                    (boost::function<void(Ogre::uint64)>) 0,
                    hlmsUnlit);
            addInteractiveElement2D(element);
        }
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
        mViewingDirectionIndicator->setMaterialName("ColorRedTransparent");
        overlayViewDirectionIndicator->add2D(mViewingDirectionIndicator);
        overlayViewDirectionIndicator->setRenderQueueGroup(254);
        overlayViewDirectionIndicator->show();

    }

    void GameState::addInteractiveElement2D(InteractiveElement2DPtr interactiveElement2D)
    {

        mInteractiveElement2DList.push_back( interactiveElement2D );
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
            mVideoDatablock[eye] = dynamic_cast<Ogre::HlmsUnlitDatablock*>(
                    hlmsUnlit->createDatablock(
                            mDatablockName[eye],
                            mDatablockName[eye],
                            Ogre::HlmsMacroblock(),
                            Ogre::HlmsBlendblock(),
                            Ogre::HlmsParamVec() ) );

            mVideoDatablock[eye]->setTexture( 0, mTextureName[eye] );
        }

        Ogre::HlmsBlendblock blendBlock = Ogre::HlmsBlendblock();
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
            RealVector vec1 = hmdConfig.initialPose;
            Ogre::Vector3 initialPose = RealVectorToVector3(vec1);
            mVRCamerasNode->setPosition(initialPose);
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
        mGraphicsSystem->mVRSceneManager->setAmbientLight(
                Ogre::ColourValue(0.6f, 0.8, 1.0),
                Ogre::ColourValue(1.0f, 0.8, 0.6),
                Ogre::Vector3(0.0f, 1.0f, 0.0f));


        mGraphicsSystem->mVRSceneManager->setVisibilityMask(0xFFFFFF30);

        //TODO: stopped working because DebugFont is not loaded anymore
        createVROverlays();
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
        finalText = mIntersectsInfoScreen ? "View Intersects" : "View not Intersects";
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
                    mEsvr2->mPoseState->getOrientation();
            orientation = orientation * Ogre::Quaternion(
                    Ogre::Degree(180), Ogre::Vector3::UNIT_Z);
            Ogre::Vector3 position = mEsvr2->mPoseState->getPosition();
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
    }

    void GameState::mousePressed( const SDL_MouseButtonEvent &arg )
    {
        if ( arg.button == SDL_BUTTON_LEFT)
        {
            setMouseRelative(true);
            mMouseManipulate = MM_ORIENTATION;
        }
        if ( arg.button == SDL_BUTTON_RIGHT)
        {
            setMouseRelative(true);
            mMouseManipulate = MM_TRANSLATION;
        }
    }

    void GameState::mouseReleased( const SDL_MouseButtonEvent &arg )
    {
        setMouseRelative(false);
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
                    if (!handled && mEsvr2->mController)
                        mEsvr2->mController->keyPressed(evt.key);
                }
                break;
            case SDL_KEYUP:
                if( !evt.key.repeat )
                {
                    bool handled = keyReleased( evt.key );
                    if (!handled && mEsvr2->mController)
                        mEsvr2->mController->keyReleased(evt.key);
                }
                break;
        }
    }

    bool GameState::keyPressed( const SDL_KeyboardEvent &arg )
    {
        bool succ = false;
        if ( arg.keysym.scancode == SDL_SCANCODE_M )
        {
            mIsUIVisible = true;
            updateOverlayElements();
            succ = true;
        }
        if( arg.keysym.scancode == SDL_SCANCODE_N && mIsUIVisible )
        {
            toggleUI();
            mUIActive = true;
            mActiveUIElement = mHoverUIElement;
            updateOverlayElements();

//            mUIStatus = UIS_ACTIVATE;
            succ = true;
        }
        return succ;
    }

    bool GameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
        if( arg.keysym.scancode == SDL_SCANCODE_ESCAPE )
        {
            mGraphicsSystem->mQuit = true;
            return true;
        }
        bool succ = false;
        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::uint32 flipMask = 0x0;
        Ogre::uint32 setMask = 0x0;
        Ogre::uint32 unsetMask = 0x0;

        if( arg.keysym.scancode == SDL_SCANCODE_Y )
        {
            mEsvr2->mVideoLoader->setDistortion(DIST_RAW);
            succ = true;
        }
        if( arg.keysym.scancode == SDL_SCANCODE_X )
        {
            mEsvr2->mVideoLoader->setDistortion(DIST_UNDISTORT);
            succ = true;
        }
//        if( arg.keysym.scancode == SDL_SCANCODE_A )
//        {
//            Ogre::Real zoom = mGraphicsSystem->getZoom();
//            zoom += arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL) ? 0.1 : -0.1;
//            mGraphicsSystem->setZoom( zoom );
//            succ = true;
//        }
        if( arg.keysym.scancode == SDL_SCANCODE_C )
        {
            mEsvr2->mVideoLoader->setDistortion(DIST_UNDISTORT_RECTIFY);
            succ = true;
        }
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
        if( arg.keysym.scancode == SDL_SCANCODE_M )
        {
            mHoverUIElement = nullptr;
            mIsUIVisible = false;
            updateOverlayElements();
        }
        if( arg.keysym.scancode == SDL_SCANCODE_N )
        {
            mActiveUIElement = nullptr;
            mUIActive = false;
            updateOverlayElements();
//            mUIStatus = UIS_NONE;
            succ = true;
        }
        return succ;
    }

    void GameState::updateOverlayElements()
    {
        for (auto it = mInteractiveElement2DList.begin();
                it != mInteractiveElement2DList.end();
                it++)
        {
            InteractiveElement2DPtr elem = *it;
            if (mUIActive)
                elem->setUIState(UIS_ACTIVATE, elem->isUVinside(mInfoScreenUV));
            else if (mIsUIVisible)
                elem->setUIState(UIS_VISIBLE, elem->isUVinside(mInfoScreenUV));
            else
                elem->setUIState(UIS_NONE, false);
        }
    }

    void GameState::toggleUI()
    {
        if (mIntersectsInfoScreen)
        {
            InteractiveElement2DPtr toggleElement =
                    findInteractiveElement2DByUV(mInfoScreenUV);
            if (toggleElement)
                toggleElement->activateToggle();
        }
    }

    void GameState::holdUI(Ogre::uint64 timeSinceLast)
    {
        if (mIntersectsInfoScreen && mActiveUIElement)
        {
            mActiveUIElement->activateHold(timeSinceLast);
            mActiveUIElement->setUIState(UIS_ACTIVATE, true);
        }
    }

    bool GameState::setDistortion(Distortion dist)
    {
        mEsvr2->mVideoLoader->setDistortion(dist);
        Ogre::uint32 setMask = 0x0;
        Ogre::uint32 unsetMask = 0x0;
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
        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask( );
        visibilityMask |= setMask;
        visibilityMask &= ~unsetMask;
        sceneManager->setVisibilityMask( visibilityMask );
    }

    void GameState::updateVRCamerasNode(void)
    {
        mVRCamerasNode->_getFullTransformUpdated();
        mVRCamerasNode->yaw(Ogre::Radian(mVRCameraNodeYaw));
        mVRCamerasNode->pitch(Ogre::Radian(mVRCameraNodePitch));
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
    }

    //-----------------------------------------------------------------------------------
    void GameState::update( Ogre::uint64 microSecsSinceLast )
    {
        updateVRCamerasNode();
        readHeadGestures();
        if (mEsvr2->mController )
            mEsvr2->mController->headPoseUpdated();
        //update Pointcloud ?
        if( mDisplayHelpMode && mGraphicsSystem->mOverlaySystem )
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
                generateDebugText( microSecsSinceLast, finalText );
                InteractiveElement2DPtr interactiveDebug =
                        findInteractiveElement2DByName("Debug");
                if (interactiveDebug)
                    interactiveDebug->setText(finalText);
            }
        }
        if (mUIActive)
        {
            holdUI(microSecsSinceLast);
        }
        else if (mIsUIVisible)
        {
            InteractiveElement2DPtr elem = nullptr;
            for (auto it = mInteractiveElement2DList.begin();
                it != mInteractiveElement2DList.end(); it++)
            {
                if((*it)->isUVinside(mInfoScreenUV))
                {
                    elem = *it;
                    break;
                }
            }
            if (mHoverUIElement != elem)
            {
                if (mHoverUIElement)
                    mHoverUIElement->setUIState(UIS_VISIBLE, false);
                if (elem)
                    elem->setUIState(UIS_VISIBLE, true);
                mHoverUIElement = elem;
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

        Ogre::Vector3 edge;
        edge = Ogre::Vector3( -mInfoScreenDim.x, mInfoScreenDim.y,0);
        mVRInfoScreen->position( edge );
        mVRInfoScreen->textureCoord(0 , 0);
        edge = Ogre::Vector3( mInfoScreenDim.x, mInfoScreenDim.y, 0 );
        mVRInfoScreen->position(edge);
        mVRInfoScreen->textureCoord(1 , 0);
        edge = Ogre::Vector3( mInfoScreenDim.x, -mInfoScreenDim.y, 0 );
        mVRInfoScreen->position(edge);
        mVRInfoScreen->textureCoord(1 , 1);
        edge = Ogre::Vector3( -mInfoScreenDim.x, -mInfoScreenDim.y, 0);
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
        Ogre::Real dist = mProjPlaneDistance[DIST_RAW];
        mInfoScreenSceneNode->translate( 0, 0, -(dist-0.01));
    }

    void GameState::createVRFloor()
    {
        Ogre::v1::MeshPtr planeMeshV1 = Ogre::v1::MeshManager::getSingleton().createPlane(
                "Plane v1",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
               Ogre::Plane( Ogre::Vector3::UNIT_Y, 0.0f ), 50.0f, 50.0f,
                                                                                           1, 1, true, 1, 4.0f, 4.0f, Ogre::Vector3::UNIT_Z,
                                                                                           Ogre::v1::HardwareBuffer::HBU_STATIC,
                                                                                           Ogre::v1::HardwareBuffer::HBU_STATIC );

        Ogre::MeshPtr planeMesh = Ogre::MeshManager::getSingleton().createByImportingV1(
                "Plane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                planeMeshV1.get(), true, true, true );

        Ogre::SceneManager *sceneManager = mGraphicsSystem->mVRSceneManager;
        Ogre::Item *item = sceneManager->createItem( planeMesh, Ogre::SCENE_DYNAMIC );

        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        item->setDatablock( "Marble" );
        Ogre::SceneNode *sceneNode = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        sceneNode->setPosition( 0, 0, 0 );
        sceneNode->attachObject( item );

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
        uv.x = (intersect_loc.x + mInfoScreenDim.x) / (2 * mInfoScreenDim.x);
        uv.y = (intersect_loc.y - mInfoScreenDim.y) / (-2 * mInfoScreenDim.y);
        mIntersectsInfoScreen = 0.0f < uv.x && uv.x < 1.0f &&  0 < uv.y && uv.y < 1.0f;
        if (mViewingDirectionIndicator)
        {
            if (mIntersectsInfoScreen)
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

    void GameState::setDebugText(Ogre::String debugText)
    {
        mDebugText = debugText;
    }
}
