#include "StereoRenderingGraphicsSystem.h"

#include "OgreSceneManager.h"
#include "OgreCamera.h"
#include "OgreRoot.h"
#include "OgreWindow.h"
// #include "Compositor/OgreCompositorManager2.h"

using namespace Demo;

namespace Demo
{
    //-------------------------------------------------------------------------------
    void StereoGraphicsSystem::createCamera(void)
    {
        //Use one node to control both cameras
        mCamerasNode = mSceneManager->getRootSceneNode( Ogre::SCENE_DYNAMIC )->
                createChildSceneNode( Ogre::SCENE_DYNAMIC );
        mCamerasNode->setName( "Cameras Node" );

        mCamerasNode->setPosition( 0, 0, 0 );

        mEyeCameras[0] = mSceneManager->createCamera( "Left Eye" );
        mEyeCameras[1] = mSceneManager->createCamera( "Right Eye" );

        const Ogre::Real eyeDistance        = 0.06f;
        const Ogre::Real eyeFocusDistance   = 0.06f;

        for( int leftOrRight = 0; leftOrRight < 2; ++leftOrRight )
        {
            const Ogre::Vector3 camPos( eyeDistance * (leftOrRight * 2 - 1), 0, 0 );
            mEyeCameras[leftOrRight]->setPosition( camPos );

            Ogre::Vector3 lookAt( eyeFocusDistance * (leftOrRight * 2 - 1), 0, -1 );
            //Ogre::Vector3 lookAt( 0, 0, 0 );

            // Look back along -Z
            mEyeCameras[leftOrRight]->lookAt( lookAt );
            mEyeCameras[leftOrRight]->setNearClipDistance( 0.2f );
            mEyeCameras[leftOrRight]->setFarClipDistance( 1000.0f );
            mEyeCameras[leftOrRight]->setAutoAspectRatio( true );

            //By default cameras are attached to the Root Scene Node.
            mEyeCameras[leftOrRight]->detachFromParent();
            mCamerasNode->attachObject( mEyeCameras[leftOrRight] );
        }

        mCamera = mEyeCameras[0];
    }

    Ogre::CompositorWorkspace* StereoGraphicsSystem::setupCompositor()
    {
        Ogre::uint8 vpModifierMask, executionMask;
        Ogre::Vector4 vpOffsetScale;

        const Ogre::IdString workspaceName( "TwoCamerasWorkspace" );
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();

        vpModifierMask  = 0x01;
        executionMask   = 0x01;
        vpOffsetScale   = Ogre::Vector4( 0.0f, 0.0f, 0.5f, 1.0f );
        mEyeWorkspaces[LEFT] = compositorManager->addWorkspace( mSceneManager,
                                                                mRenderWindow->getTexture(),
                                                                mEyeCameras[LEFT], workspaceName,
                                                                true, -1, (Ogre::UavBufferPackedVec*)0,
                                                                (Ogre::ResourceLayoutMap*)0,
                                                                (Ogre::ResourceAccessMap*)0,
                                                                vpOffsetScale,
                                                                vpModifierMask,
                                                                executionMask );

        vpModifierMask  = 0x02;
        executionMask   = 0x02;
        vpOffsetScale   = Ogre::Vector4( 0.5f, 0.0f, 0.5f, 1.0f );
        mEyeWorkspaces[LEFT] = compositorManager->addWorkspace( mSceneManager,
                                                                mRenderWindow->getTexture(),
                                                                mEyeCameras[RIGHT], workspaceName,
                                                                true, -1, (Ogre::UavBufferPackedVec*)0,
                                                                (Ogre::ResourceLayoutMap*)0,
                                                                (Ogre::ResourceAccessMap*)0,
                                                                vpOffsetScale,
                                                                vpModifierMask,
                                                                executionMask);
        return mEyeWorkspaces[0];
    }

    StereoGraphicsSystem::StereoGraphicsSystem( GameState* gameState, WorkspaceType wsType ) :
        GraphicsSystem( gameState, "../Data/" )
    {
        mWorkSpaceType = wsType;
    }

    StereoGraphicsSystem::~StereoGraphicsSystem()
    {
    //             delete mEyeCameras;
    }
}
