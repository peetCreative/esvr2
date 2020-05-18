
#include "StereoRenderingGameState.h"
#include "StereoRendering.h"
#include "CameraController.h"
#include "GraphicsSystem.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsUnlit.h"
#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"

#include "OgreCamera.h"

using namespace Demo;

namespace Demo
{
    StereoRenderingGameState::StereoRenderingGameState(
            const Ogre::String &helpDescription,
            bool isStereo, Ogre::VrData *vrData ) :
        TutorialGameState( helpDescription ),
        mSceneNodeCube( nullptr ),
        mVideoDatablock{ nullptr, nullptr },
        mProjectionRectangle{ nullptr, nullptr },
        mSceneNodeCamera( nullptr ),
        mSceneNodeLight( nullptr ),
        mVrData( vrData ),
        mIsStereo( isStereo ),
        mEyeNum( isStereo ? 2 : 1 ),
        mProjPlaneDistance( 0 ),
        mLeft{ 0, 0 },
        mRight{ 0, 0 },
        mTop{ 0, 0 },
        mBottom{ 0, 0 }
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

    void StereoRenderingGameState::calcAlign(CameraConfig &cameraConfig,
                                         Ogre::Real projPlaneDistance)
    {
        mProjPlaneDistance = projPlaneDistance;
        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            Ogre::Real width = cameraConfig.width[eye];
            Ogre::Real height = cameraConfig.height[eye];
            Ogre::Real f_x = cameraConfig.f_x[eye];
            Ogre::Real f_y = cameraConfig.f_y[eye];
            Ogre::Real c_x = cameraConfig.c_x[eye];
            Ogre::Real c_y = cameraConfig.c_y[eye];
            //in xy left is negativ
            mLeft[eye] = -c_x * mProjPlaneDistance / f_x;
            mRight[eye] = -( c_x - width ) * mProjPlaneDistance / f_x;
            mTop[eye] = c_y * mProjPlaneDistance / f_y;
            mBottom[eye] = ( c_y - height ) * mProjPlaneDistance / f_y;
        }
    }

    void StereoRenderingGameState::createCube()
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        Ogre::Item *mCube = sceneManager->createItem(
            "Cube_d.mesh",
            Ogre::ResourceGroupManager::
            AUTODETECT_RESOURCE_GROUP_NAME,
            Ogre::SCENE_DYNAMIC );

        mCube->setVisibilityFlags( 0x000000003 );
        mSceneNodeCube = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );

        mSceneNodeCube->setPosition( 0, 0, 0 );

        mSceneNodeCube->scale(0.25, 0.25, 0.25);
        mSceneNodeCube->attachObject( mCube );
    }

    void StereoRenderingGameState::createProjectionPlanes()
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
        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            mProjectionRectangle[eye] = sceneManager->createManualObject();

            mProjectionRectangle[eye]->begin(mDatablockName[eye], Ogre::OT_TRIANGLE_LIST);

            // Back
            edge = mVrData->mHeadToEye[eye] *
                Ogre::Vector4( mLeft[eye], mTop[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position( edge.xyz() );
            mProjectionRectangle[eye]->textureCoord(0 , 0);
            edge = mVrData->mHeadToEye[eye] *
                Ogre::Vector4( mRight[eye], mTop[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 0);
            edge = mVrData->mHeadToEye[eye] *
                Ogre::Vector4( mRight[eye], mBottom[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(1 , 1);
            edge = mVrData->mHeadToEye[eye] *
                Ogre::Vector4( mLeft[eye], mBottom[eye],
                               -mProjPlaneDistance, 1.0f );
            mProjectionRectangle[eye]->position(edge.xyz());
            mProjectionRectangle[eye]->textureCoord(0 , 1);
            mProjectionRectangle[eye]->quad(3, 2, 1, 0);

            mProjectionRectangle[eye]->end();


            mSceneNodeCamera->attachObject(mProjectionRectangle[eye]);
        }

        if ( mIsStereo )
        {
            mProjectionRectangle[LEFT]->setVisibilityFlags(0x000000001);
            mProjectionRectangle[RIGHT]->setVisibilityFlags(0x000000002);
        }
    }

    //-----------------------------------------------------------------------------------
    void StereoRenderingGameState::createScene01(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

        createCube();

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

        Ogre::Light *light = sceneManager->createLight();
        mSceneNodeLight = sceneManager->getRootSceneNode()->createChildSceneNode();
        mSceneNodeLight->attachObject( light );
        light->setPowerScale( Ogre::Math::PI ); //Since we don't do HDR, counter the PBS' division by PI
        light->setType( Ogre::Light::LT_DIRECTIONAL );
        light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );

        mCameraController = new CameraController( mGraphicsSystem, true );

        TutorialGameState::createScene01();
    }
    //-----------------------------------------------------------------------------------
    void StereoRenderingGameState::update( float timeSinceLast )
    {
        //update Pointcloud

        TutorialGameState::update( timeSinceLast );
    }
}
