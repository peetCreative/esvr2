#include "Esvr2GameState.h"

#include "Esvr2StereoRendering.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2PointCloud.h"

#include "CameraController.h"
#include "GraphicsSystem.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsUnlit.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"
#include "OgreBillboard.h"
#include "OgreBillboardSet.h"
#include "OgreEntity.h"
#include "OgreCamera.h"

namespace esvr2
{
    GameState::GameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData ) :
        TutorialGameState( helpDescription ),
        mStereoGraphicsSystem( nullptr ),
        mVideoDatablock{ nullptr, nullptr },
        mProjectionRectangle{ nullptr, nullptr },
        mTooltips( nullptr ),
        mPointCloud( nullptr ),
        mSceneNodeLight( nullptr ),
        mSceneNodeCamera( nullptr ),
        mSceneNodePointCloud( nullptr ),
        mSceneNodeTooltips( nullptr ),
        mSceneNodeMesh( nullptr ),
        mVrData( vrData ),
        mIsStereo( isStereo ),
        mEyeNum( isStereo ? 2 : 1 ),
        mProjPlaneDistance( 0 ),
        mLeft{ 0, 0 },
        mRight{ 0, 0 },
        mTop{ 0, 0 },
        mBottom{ 0, 0 },
        mScale( 1.0f )
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

    void GameState::calcAlign(StereoCameraConfig &cameraConfig,
                                         Ogre::Real projPlaneDistance )
    {
        mProjPlaneDistance = projPlaneDistance;
        if ( cameraConfig.leftToRight )
            mScale = mVrData->mLeftToRight.length() / cameraConfig.leftToRight;
        for( size_t eye = 0; eye < mEyeNum * 2; eye++ )
        {
            CameraConfig cfg = cameraConfig.cfg [eye%2];
            Ogre::Real width = cfg.width;
            Ogre::Real height = cfg.height;

            Ogre::Real f_x, f_y, c_x, c_y;
            if (eye < mEyeNum)
            {
                f_x = cfg.K[0];
                f_y = cfg.K[4];
                c_x = cfg.K[2];
                c_y = cfg.K[5];
            }
            else
            {
                f_x = cfg.P[0];
                f_y = cfg.P[5];
                c_x = cfg.P[2];
                c_y = cfg.P[6];
            }

            //in xy left is negativ
            mLeft[eye] = -c_x * mProjPlaneDistance / f_x;
            mRight[eye] = -( c_x - width ) * mProjPlaneDistance / f_x;
            mTop[eye] = c_y * mProjPlaneDistance / f_y;
            mBottom[eye] = ( c_y - height ) * mProjPlaneDistance / f_y;
        }
    }

    void GameState::createProjectionPlanes()
    {
        bool alldata = mVrData->mHeadToEye[LEFT] != Ogre::Matrix4::IDENTITY &&
            mProjPlaneDistance &&
            mLeft[LEFT] && mRight[LEFT] && mTop[LEFT] && mBottom[LEFT] &&
            ( !mIsStereo ||
                ( mLeft[RIGHT] && mRight[RIGHT] &&
                 mTop[RIGHT] && mBottom[RIGHT] )
            );
        if ( !alldata )
            return;

        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

        Ogre::Vector4 edge;
        //we need to create two planes for raw and recitified
        for( size_t eye = 0; eye < 2 * mEyeNum; eye++ )
        {
            mProjectionRectangle[eye] = sceneManager->createManualObject();

            mProjectionRectangle[eye]->begin(
                mDatablockName[eye%2], Ogre::OT_TRIANGLE_LIST);

            Ogre::Matrix4 eyeToHead;
            if ( mEyeNum == 2 )
                eyeToHead = mVrData->mHeadToEye[eye%2].inverse();
            else
                eyeToHead = Ogre::Matrix4::IDENTITY;

            // Back
            edge = eyeToHead *
                Ogre::Vector4( mLeft[eye], mTop[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position( edge.xyz() );
            mProjectionRectangle[eye]->textureCoord(0 , 0);
            edge = eyeToHead *
                Ogre::Vector4( mRight[eye], mTop[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 0);
            edge = eyeToHead *
                Ogre::Vector4( mRight[eye], mBottom[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 1);
            edge = eyeToHead *
                Ogre::Vector4( mLeft[eye], mBottom[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(0 , 1);
            mProjectionRectangle[eye]->quad(0, 1, 2, 3);
            mProjectionRectangle[eye]->quad(3, 2, 1, 0);

            mProjectionRectangle[eye]->end();


            mSceneNodeCamera->attachObject(mProjectionRectangle[eye]);
            mProjectionRectangle[eye]->setVisibilityFlags( 0x10 << eye );
        }
    }

    void GameState::createTooltips( void )
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        mSceneNodeTooltips = sceneManager->getRootSceneNode(
                Ogre::SCENE_DYNAMIC )->
                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mSceneNodeTooltips->setPosition( 0, 0, -0.05 * mScale );
        mTooltips = sceneManager->createBillboardSet();
        mTooltips->beginBillboards(1);
        Ogre::v1::Billboard* b = mTooltips->createBillboard(
            0.0, 0.0, 0.0);
        b->setDimensions(0.001 * mScale, 0.001 * mScale);
        b->setColour(Ogre::ColourValue::Red);
        mTooltips->endBillboards();
        mTooltips->setVisibilityFlags( 0x1 );
        mSceneNodeTooltips->attachObject( mTooltips );
    }


    void GameState::createMesh()
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        Ogre::Item *mCube = sceneManager->createItem(
            "Cube_d.mesh",
            Ogre::ResourceGroupManager::
            AUTODETECT_RESOURCE_GROUP_NAME,
            Ogre::SCENE_DYNAMIC );

        mCube->setVisibilityFlags( 0x1 << 1 );
        mSceneNodeMesh = sceneManager
            ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
            ->createChildSceneNode( Ogre::SCENE_DYNAMIC );

        mSceneNodeMesh->setPosition( 0, 0, -1.0 );

        mSceneNodeMesh->scale(0.25, 0.25, 0.25);
        mSceneNodeMesh->attachObject( mCube );
    }

    void GameState::createPointCloud( void )
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
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

        Ogre::v1::Entity *pcEnt = sceneManager->createEntity(
            mPointCloud->getMeshPtr());

        pcEnt->setMaterialName("Pointcloud");

        mSceneNodePointCloud = sceneManager
            ->getRootSceneNode( Ogre::SCENE_DYNAMIC )
            ->createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mSceneNodePointCloud->setPosition( 0, 0, -1.0 );
        mSceneNodePointCloud->attachObject( pcEnt );
        pcEnt->setVisibilityFlags( 0x1 << 2 );
    }

    //-----------------------------------------------------------------------------------
    void GameState::createScene01(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();


        Ogre::HlmsManager *hlmsManager = mGraphicsSystem->getRoot()->getHlmsManager();
        Ogre::HlmsUnlit *hlmsUnlit = static_cast<Ogre::HlmsUnlit*>( hlmsManager->getHlms(Ogre::HLMS_UNLIT) );

        Ogre::SceneManager::SceneNodeList scnlist =
            sceneManager->findSceneNodes("Cameras Node");
        if( !scnlist.empty() )
        {
            mSceneNodeCamera = scnlist.at(0);
        }
        else
        {
            mSceneNodeCamera = sceneManager->getRootSceneNode(
                Ogre::SCENE_DYNAMIC )->
                    createChildSceneNode( Ogre::SCENE_DYNAMIC );
        }


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

        createProjectionPlanes();
        createTooltips();
        createPointCloud();
        createMesh();

        Ogre::Light *light = sceneManager->createLight();
        mSceneNodeLight = sceneManager->getRootSceneNode()->createChildSceneNode();
        mSceneNodeLight->attachObject( light );
        light->setPowerScale( Ogre::Math::PI ); //Since we don't do HDR, counter the PBS' division by PI
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        mCameraController = new Demo::CameraController( mGraphicsSystem, true );

        sceneManager->setVisibilityMask(0xFFFFFF30);
        TutorialGameState::createScene01();
    }
    //-----------------------------------------------------------------------------------
    void GameState::update( float timeSinceLast )
    {
        //update Pointcloud
        TutorialGameState::update( timeSinceLast );
    }

    void GameState::keyReleased( const SDL_KeyboardEvent &arg )
    {
        if( arg.keysym.scancode == SDL_SCANCODE_ESCAPE )
        {
            mGraphicsSystem->setQuit();
            return;
        }
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        Ogre::uint32 flipMask = 0x0;
        Ogre::uint32 setMask = 0x0;
        Ogre::uint32 unsetMask = 0x0;

        //strg + 1 projectionplanes

        // stop Video
        if( arg.keysym.scancode == SDL_SCANCODE_X
        )
        {
            mStereoGraphicsSystem->itterateDistortion();
            Distortion dist = mStereoGraphicsSystem->getDistortion();
            if (dist == UNDISTORT_RECTIFY)
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
        if( arg.keysym.scancode == SDL_SCANCODE_1 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            Ogre::uint32 visibilityMask = sceneManager->getVisibilityMask();
            if (visibilityMask & 0xF0)
                unsetMask = 0xF0;
            else
            {
                Distortion dist = mStereoGraphicsSystem->getDistortion();
                if (dist == UNDISTORT_RECTIFY)
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
        }
        //strg + 2 tool tip
        if( arg.keysym.scancode == SDL_SCANCODE_2 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1;
        }
        //strg + 4 mesh
        if( arg.keysym.scancode == SDL_SCANCODE_3 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1 << 1;
        }
        //strg + 3 point cloud
        if( arg.keysym.scancode == SDL_SCANCODE_4 &&
            (arg.keysym.mod & (KMOD_LCTRL|KMOD_RCTRL)) )
        {
            flipMask = 0x1 << 2;
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
            mStereoGraphicsSystem->toggleShowVideo();
        }

        TutorialGameState::keyReleased( arg );
    }
}
