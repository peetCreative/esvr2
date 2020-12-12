#include "Esvr2GameState.h"

#include "Esvr2.h"
#include "Esvr2PointCloud.h"

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
#include "Overlay/OgreOverlay.h"
#include "Overlay/OgreOverlayManager.h"
#include "Overlay/OgreOverlayContainer.h"
#include "Overlay/OgreTextAreaOverlayElement.h"

namespace esvr2
{
    GameState::GameState(Esvr2 *esvr2):
            mEsvr2(esvr2),
            mGraphicsSystem( nullptr ),
            mVideoDatablock{ nullptr, nullptr },
            mProjectionRectangle{ nullptr, nullptr, nullptr, nullptr },
            mAxis( nullptr ),
            mAxisCameras( nullptr ),
            mTooltips( nullptr ),
            mPointCloud( nullptr ),
            mVRSceneNodeLight( nullptr ),
            mVRSceneNodeMesh( nullptr ),
            mVRSceneNodesProjectionPlaneRaw(nullptr),
            mVRSceneNodesProjectionPlaneRect(nullptr),
            mLaparoscopeSceneNodePointCloud( nullptr ),
            mLaparoscopeSceneNodeTooltips( nullptr ),
            mIsStereo( esvr2->mConfig->isStereo ),
            mEyeNum( esvr2->mConfig->isStereo ? 2 : 1 ),
            mProjPlaneDistance{ 0, 0, 0, 0 },
            mLeft{ 0, 0 },
            mRight{ 0, 0 },
            mTop{ 0, 0 },
            mBottom{ 0, 0 },
            mScale( 1.0f ),
            mDisplayHelpMode(true),
            mDebugText(nullptr),
            mDebugTextShadow(nullptr),
            mHelpDescription("")
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

    void GameState::calcAlign()
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
            Ogre::Real f_x, f_y, c_x, c_y;
            if (eye < mEyeNum)
            {
                f_x = cfg->K[0];
                f_y = cfg->K[4];
//                 c_x = width/2;
//                 c_y = height/2;
                c_x = cfg->K[2];
                c_y = cfg->K[5];
                mProjPlaneDistance[eye] = projPlaneDistance;
            }
            else
            {
                f_x = cfg->P[0];
                f_y = cfg->P[5];
//                 c_x = width/2;
//                 c_y = height/2;
                c_x = cfg->P[2];
                c_y = cfg->P[6];

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

    void GameState::createProjectionPlanes()
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

        mVRSceneNodesProjectionPlaneRaw =
                mGraphicsSystem->mVRSceneManager
                ->getRootSceneNode()
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRaw->setName("VR Node Projection Plane Raw");

        mVRSceneNodesProjectionPlaneRect =
                mGraphicsSystem->mVRSceneManager
                ->getRootSceneNode()
                ->createChildSceneNode(Ogre::SCENE_DYNAMIC);
        mVRSceneNodesProjectionPlaneRect->setName("VR Node Projection Plane Rect");
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
                               -mProjPlaneDistance[eye], 1.0f );
            mProjectionRectangle[eye]->position( edge.xyz() );
            mProjectionRectangle[eye]->textureCoord(0 , 0);
            edge = Ogre::Vector4( mRight[eye], mTop[eye],
                               -mProjPlaneDistance[eye], 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 0);
            edge = Ogre::Vector4( mRight[eye], mBottom[eye],
                               -mProjPlaneDistance[eye], 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 1);
            edge = Ogre::Vector4( mLeft[eye], mBottom[eye],
                               -mProjPlaneDistance[eye], 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(0 , 1);
            mProjectionRectangle[eye]->quad(0, 1, 2, 3);
            mProjectionRectangle[eye]->quad(3, 2, 1, 0);

            mProjectionRectangle[eye]->end();

            sceneNodesProjectionPlanes[eye%2]->attachObject(mProjectionRectangle[eye]);
//            mProjectionRectangle[eye]->setVisibilityFlags( 0x10 << eye );
        }
    }

    Ogre::ManualObject *GameState::createAxisIntern( Ogre::SceneManager *sceneManager )
    {
        Ogre::ManualObject *axis =
                sceneManager->createManualObject();
        Ogre::Real diam = 0.1f;
        Ogre::Real len = 1.0f;
        axis->begin("Red", Ogre::OT_TRIANGLE_LIST);
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
        axis->begin("Green", Ogre::OT_TRIANGLE_LIST);
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
        axis->begin("Blue", Ogre::OT_TRIANGLE_LIST);
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

    void GameState::createAxis(void)
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

    void GameState::createVRAxis(void)
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
        mGraphicsSystem->mVRCamerasNode->attachObject(mAxisVRCameras);
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

//        mCube->setVisibilityFlags( 0x1 << 2 );
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

    void GameState::loadDatablocks()
    {
        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = dynamic_cast<Ogre::HlmsUnlit*>(
                hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        Ogre::HlmsUnlitDatablock* colourdatablock =
                static_cast<Ogre::HlmsUnlitDatablock*>(
                        hlmsUnlit->createDatablock(
                                "Red",
                                "Red",
                                Ogre::HlmsMacroblock(),
                                Ogre::HlmsBlendblock(),
                                Ogre::HlmsParamVec() ) );
        colourdatablock->setUseColour(true);
        colourdatablock->setColour( Ogre::ColourValue(1,0,0,1));
        colourdatablock =
                static_cast<Ogre::HlmsUnlitDatablock*>(
                        hlmsUnlit->createDatablock(
                                "Green",
                                "Green",
                                Ogre::HlmsMacroblock(),
                                Ogre::HlmsBlendblock(),
                                Ogre::HlmsParamVec() ) );
        colourdatablock->setUseColour(true);
        colourdatablock->setColour( Ogre::ColourValue(0,1,0,1));
        colourdatablock =
                static_cast<Ogre::HlmsUnlitDatablock*>(
                        hlmsUnlit->createDatablock(
                                "Blue",
                                "Blue",
                                Ogre::HlmsMacroblock(),
                                Ogre::HlmsBlendblock(),
                                Ogre::HlmsParamVec() ) );
        colourdatablock->setUseColour(true);
        colourdatablock->setColour( Ogre::ColourValue(0,0,1,1));
        const Ogre::HlmsManager::HlmsDatablockMap datablocks = hlmsManager->getDatablocks();
        for(auto d = datablocks.begin(); d != datablocks.end(); d++ )
        {
            LOG << "HlmsDatablock: " << d->second->getName().getFriendlyText() << LOGEND;
        }
    }

    void GameState::createLaparoscopeScene()
    {
        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = dynamic_cast<Ogre::HlmsUnlit*>(
                hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            mVideoDatablock[eye] = static_cast<Ogre::HlmsUnlitDatablock*>(
                    hlmsUnlit->createDatablock(
                            mDatablockName[eye],
                            mDatablockName[eye],
                            Ogre::HlmsMacroblock(),
                            Ogre::HlmsBlendblock(),
                            Ogre::HlmsParamVec() ) );

            mVideoDatablock[eye]->setTexture( 0, mTextureName[eye] );
        }
        //Follow the tip make the upper Hemisphere little more blue
        mGraphicsSystem->mLaparoscopeSceneManager->setAmbientLight(
                Ogre::ColourValue(0.6f, 0.8, 1.0),
                Ogre::ColourValue(1.0f, 0.8, 0.6),
                Ogre::Vector3(0.0f, 1.0f, 0.0f));

//        createAxis();
//        createTooltips();
//         createPointCloud();

    }

    //-----------------------------------------------------------------------------------
    void GameState::createVRScene(void)
    {
        calcAlign();

        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->mRoot->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = static_cast<Ogre::HlmsUnlit*>( hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

//         createProjectionRectangle2D();
         createProjectionPlanes();
        createMesh();
        createVRAxis();

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

        //TODO: write camera Controller
        //mCameraController = new CameraController( mGraphicsSystem, true );

//        mGraphicsSystem->mVRSceneManager->setVisibilityMask(0xFFFFFF30);

        //FROM
        //TutorialGameState::createScene01();
        Ogre::v1::OverlayManager &overlayManager =
                Ogre::v1::OverlayManager::getSingleton();
        Ogre::v1::Overlay *overlay = overlayManager.create( "DebugText" );

        Ogre::v1::OverlayContainer *panel = static_cast<Ogre::v1::OverlayContainer*>(
                overlayManager.createOverlayElement("Panel", "DebugPanel"));
        mDebugText = static_cast<Ogre::v1::TextAreaOverlayElement*>(
                overlayManager.createOverlayElement( "TextArea", "DebugText" ) );
        mDebugText->setFontName( "DebugFont" );
        mDebugText->setCharHeight( 0.025f );

        mDebugTextShadow= static_cast<Ogre::v1::TextAreaOverlayElement*>(
                overlayManager.createOverlayElement( "TextArea", "0DebugTextShadow" ) );
        mDebugTextShadow->setFontName( "DebugFont" );
        mDebugTextShadow->setCharHeight( 0.025f );
        mDebugTextShadow->setColour( Ogre::ColourValue::Black );
        mDebugTextShadow->setPosition( 0.002f, 0.002f );

        panel->addChild( mDebugTextShadow );
        panel->addChild( mDebugText );
        overlay->add2D( panel );
        overlay->show();

    }

    //-----------------------------------------------------------------------------------
    void GameState::generateDebugText( float timeSinceLast, Ogre::String &outText )
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

        Ogre::String finalText;
        finalText.reserve( 128 );
        finalText  = "Frame time:\t";
        finalText += Ogre::StringConverter::toString( timeSinceLast * 1000.0f );
        finalText += " ms\n";
        finalText += "Frame FPS:\t";
        finalText += Ogre::StringConverter::toString( 1.0f / timeSinceLast );
        finalText += "\nAvg time:\t";
        finalText += Ogre::StringConverter::toString( frameStats->getAvgTime() );
        finalText += " ms\n";
        finalText += "Avg FPS:\t";
        finalText += Ogre::StringConverter::toString( 1000.0f / frameStats->getAvgTime() );
        finalText += "\n\nPress F1 to toggle help";

        outText.swap( finalText );

        mDebugText->setCaption( finalText );
        mDebugTextShadow->setCaption( finalText );
    }


    //-----------------------------------------------------------------------------------
    void GameState::update( float timeSinceLast )
    {
        //update Pointcloud ?
        if( mDisplayHelpMode )
        {
            //Show FPS
            Ogre::String finalText;
            generateDebugText( timeSinceLast, finalText );
            mDebugText->setCaption( finalText );
            mDebugTextShadow->setCaption( finalText );
        }

        //TODO: move Camera here
//        if( mCameraController )
//            mCameraController->update( timeSinceLast );
    }

//    void GameState::keyReleased( const SDL_KeyboardEvent &arg )
//    {
//        if( arg.keysym.scancode == SDL_SCANCODE_ESCAPE )
//        {
//            mGraphicsSystem->setQuit();
//            return;
//        }
//        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
//        Ogre::uint32 flipMask = 0x0;
//        Ogre::uint32 setMask = 0x0;
//        Ogre::uint32 unsetMask = 0x0;
//
//        if( arg.keysym.scancode == SDL_SCANCODE_Y )
//        {
//            mGraphicsSystem->setDistortion(DIST_RAW);
//        }
//        if( arg.keysym.scancode == SDL_SCANCODE_X )
//        {
//            mGraphicsSystem->setDistortion(DIST_UNDISTORT);
//        }
//        if( arg.keysym.scancode == SDL_SCANCODE_A )
//        {
//            Ogre::Real zoom = mGraphicsSystem->getZoom();
//            zoom += arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL) ? 0.1 : -0.1;
//            mGraphicsSystem->setZoom( zoom );
//        }
//        if( arg.keysym.scancode == SDL_SCANCODE_C )
//        {
//            mGraphicsSystem->setDistortion(DIST_UNDISTORT_RECTIFY);
//        }
//        if( arg.keysym.scancode == SDL_SCANCODE_1 &&
//            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
//        {
//            Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask();
//            if (visibilityMask & 0xF0)
//                unsetMask = 0xF0;
//            else
//            {
//                Distortion dist = mEsvr2->mVideoLoader->getDistortion();
//                if (dist == DIST_UNDISTORT_RECTIFY)
//                {
//                    setMask = 0x40 | 0x80;
//                    unsetMask = 0x10 | 0x20;
//                }
//                else
//                {
//                    setMask = 0x10 | 0x20;
//                    unsetMask = 0x40 | 0x80;
//                }
//            }
//        }
//        //strg + 2 axis
//        if( arg.keysym.scancode == SDL_SCANCODE_2 &&
//            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
//        {
//            flipMask = 0x1;
//        }
//        //strg + 4 mesh
//        if( arg.keysym.scancode == SDL_SCANCODE_3 &&
//            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
//        {
//            flipMask = 0x1 << 1;
//        }
//        //strg + 3 point cloud
//        if( arg.keysym.scancode == SDL_SCANCODE_4 &&
//            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
//        {
//            flipMask = 0x1 << 2;
//        }
//
//        //TODO: It's too complicated for just flipping certain bits
//        Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask();
//        visibilityMask &= ~flipMask;
//        visibilityMask |= ~sceneManager->getVisibilityMask() & flipMask;
//        visibilityMask |= setMask;
//        visibilityMask &= ~unsetMask;
//        sceneManager->setVisibilityMask( visibilityMask );
//
//        // stop Video
//        if( arg.keysym.scancode == SDL_SCANCODE_SPACE )
//        {
//            mGraphicsSystem->toggleShowVideo();
//        }
//
//        TutorialGameState::keyReleased( arg );
//    }
}
