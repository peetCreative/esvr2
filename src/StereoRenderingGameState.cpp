
#include "StereoRenderingGameState.h"
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
    StereoRenderingGameState::StereoRenderingGameState( const Ogre::String &helpDescription ) :
        TutorialGameState( helpDescription )
    {
        memset( mSceneNode, 0, sizeof(mSceneNode) );
    }

    void StereoRenderingGameState::createCube()
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();
        Ogre::Item *mCube = sceneManager->createItem(
            "Cube_d.mesh",
            Ogre::ResourceGroupManager::
            AUTODETECT_RESOURCE_GROUP_NAME,
            Ogre::SCENE_DYNAMIC );

        mCube->setVisibilityFlags( 0x000000002 );
        Ogre::SceneNode *mSceneNodeCube = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
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

        Ogre::String datablockName = "Video";
        Ogre::HlmsUnlitDatablock *datablock = static_cast<Ogre::HlmsUnlitDatablock*>(
            hlmsUnlit->createDatablock(
                datablockName,
                datablockName,
                Ogre::HlmsMacroblock(),
                Ogre::HlmsBlendblock(),
                Ogre::HlmsParamVec() ) );

        datablock->setTexture( 0, "VideoTexture" );

        Ogre::ManualObject * manualObject = sceneManager->createManualObject();

        manualObject->begin(datablockName, Ogre::OT_TRIANGLE_LIST);

        // Back
        manualObject->position(-0.3f, -0.3f, -1.0f);
        manualObject->textureCoord(0 , 0);
        manualObject->position(-0.3f, 0.3f, -1.0f);
        manualObject->textureCoord(0 , 1);
        manualObject->position(0.3f, 0.3f, -1.0f);
        manualObject->textureCoord(1 , 1);
        manualObject->position(0.3f, -0.3f, -1.0f);
        manualObject->textureCoord(1 , 0);
        manualObject->quad(3, 2, 1, 0);

        manualObject->end();

        Ogre::SceneNode *scn = nullptr;
        Ogre::SceneManager::SceneNodeList scnlist = sceneManager->findSceneNodes("Cameras Node");
        if( !scnlist.empty() )
        {
            scn = scnlist.at(0);
        }
        else
        {
            scn = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                                     createChildSceneNode( Ogre::SCENE_DYNAMIC );
        }
        scn->attachObject(manualObject);

        Ogre::Light *light = sceneManager->createLight();
        Ogre::SceneNode *lightNode = sceneManager->getRootSceneNode()->createChildSceneNode();
        lightNode->attachObject( light );
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
