
#include "StereoRenderingGameState.h"
#include "CameraController.h"
#include "GraphicsSystem.h"

#include "OgreSceneManager.h"
#include "OgreItem.h"

#include "OgreCamera.h"

using namespace Demo;

namespace Demo
{
    StereoRenderingGameState::StereoRenderingGameState( const Ogre::String &helpDescription ) :
        TutorialGameState( helpDescription )
    {
        memset( mSceneNode, 0, sizeof(mSceneNode) );
    }
    //-----------------------------------------------------------------------------------
    void StereoRenderingGameState::createScene01(void)
    {
        Ogre::SceneManager *sceneManager = mGraphicsSystem->getSceneManager();

        const float armsLength = 2.5f;

        for( int i=0; i<4; ++i )
        {
            for( int j=0; j<4; ++j )
            {
                Ogre::Item *item = sceneManager->createItem( "Cube_d.mesh",
                                                             Ogre::ResourceGroupManager::
                                                             AUTODETECT_RESOURCE_GROUP_NAME,
                                                             Ogre::SCENE_DYNAMIC );

                size_t idx = i * 4 + j;

                mSceneNode[idx] = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                        createChildSceneNode( Ogre::SCENE_DYNAMIC );

                mSceneNode[idx]->setPosition( (i - 1.5f) * armsLength,
                                              0.0f,
                                              (j - 1.5f) * armsLength );

                mSceneNode[idx]->roll( Ogre::Radian( idx ) );

                mSceneNode[idx]->attachObject( item );
            }
        }

        Ogre::ManualObject * manualObject = sceneManager->createManualObject();

        manualObject->begin("BaseWhite", Ogre::OT_LINE_LIST);

        // Back
        manualObject->position(0.0f, 0.0f, 0.0f);
        manualObject->position(0.0f, 1.0f, 0.0f);
        manualObject->line(0, 1);

        manualObject->position(0.0f, 1.0f, 0.0f);
        manualObject->position(1.0f, 1.0f, 0.0f);
        manualObject->line(2, 3);

        manualObject->position(1.0f, 1.0f, 0.0f);
        manualObject->position(1.0f, 0.0f, 0.0f);
        manualObject->line(4, 5);

        manualObject->position(1.0f, 0.0f, 0.0f);
        manualObject->position(0.0f, 0.0f, 0.0f);
        manualObject->line(6, 7);

        // Front
        manualObject->position(0.0f, 0.0f, 1.0f);
        manualObject->position(0.0f, 1.0f, 1.0f);
        manualObject->line(8, 9);

        manualObject->position(0.0f, 1.0f, 1.0f);
        manualObject->position(1.0f, 1.0f, 1.0f);
        manualObject->line(10, 11);

        manualObject->position(1.0f, 1.0f, 1.0f);
        manualObject->position(1.0f, 0.0f, 1.0f);
        manualObject->line(12, 13);

        manualObject->position(1.0f, 0.0f, 1.0f);
        manualObject->position(0.0f, 0.0f, 1.0f);
        manualObject->line(14, 15);

        // Sides
        manualObject->position(0.0f, 0.0f, 0.0f);
        manualObject->position(0.0f, 0.0f, 1.0f);
        manualObject->line(16, 17);

        manualObject->position(0.0f, 1.0f, 0.0f);
        manualObject->position(0.0f, 1.0f, 1.0f);
        manualObject->line(18, 19);

        manualObject->position(1.0f, 0.0f, 0.0f);
        manualObject->position(1.0f, 0.0f, 1.0f);
        manualObject->line(20, 21);

        manualObject->position(1.0f, 1.0f, 0.0f);
        manualObject->position(1.0f, 1.0f, 1.0f);
        manualObject->line(22, 23);

        manualObject->end();

        Ogre::SceneNode *sceneNodeLines = sceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                                     createChildSceneNode( Ogre::SCENE_DYNAMIC );
        sceneNodeLines->attachObject(manualObject);
        sceneNodeLines->scale(4.0f, 4.0f, 4.0f);
        sceneNodeLines->translate(-4.5f, -1.5f, 0.0f, Ogre::SceneNode::TS_LOCAL);

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
        for( int i=0; i<16; ++i )
            mSceneNode[i]->yaw( Ogre::Radian(timeSinceLast * i * 0.25f) );

        TutorialGameState::update( timeSinceLast );
    }
}
