
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
            bool isStereo ) :
        TutorialGameState( helpDescription ),
        mSceneNodeCube( nullptr ),
        mVideoDatablock{ nullptr, nullptr },
        mProjectionRectangle{ nullptr, nullptr },
        mSceneNodeCamera( nullptr ),
        mSceneNodeLight( nullptr ),
        mIsStereo( isStereo ),
        mEyeNum( isStereo ? 2 : 1 )
    {
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

        Ogre::String categories[2];
        if( mIsStereo )
        {
            categories[LEFT] = "Left";
            categories[RIGHT] = "Right";
        }
        else
        {
            categories[0] = "Mono";
        }

        for( size_t eye = 0; eye < mEyeNum; eye++ )
        {
            Ogre::String textureName = "VideoTexture" + categories[eye];
            Ogre::String datablockName = "Video" + categories[eye];
            mVideoDatablock[eye] = static_cast<Ogre::HlmsUnlitDatablock*>(
                hlmsUnlit->createDatablock(
                    datablockName,
                    datablockName,
                    Ogre::HlmsMacroblock(),
                    Ogre::HlmsBlendblock(),
                    Ogre::HlmsParamVec() ) );

            mVideoDatablock[eye]->setTexture( 0, textureName );

            mProjectionRectangle[eye] = sceneManager->createManualObject();

            mProjectionRectangle[eye]->begin(datablockName, Ogre::OT_TRIANGLE_LIST);

            // Back
            mProjectionRectangle[eye]->position(-0.3f, 0.3f, -1.0f);
            mProjectionRectangle[eye]->textureCoord(0 , 0);
            mProjectionRectangle[eye]->position(0.3f, 0.3f, -1.0f);
            mProjectionRectangle[eye]->textureCoord(1 , 0);
            mProjectionRectangle[eye]->position(0.3f, -0.3f, -1.0f);
            mProjectionRectangle[eye]->textureCoord(1 , 1);
            mProjectionRectangle[eye]->position(-0.3f, -0.3f, -1.0f);
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
