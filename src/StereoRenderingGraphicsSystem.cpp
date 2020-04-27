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
        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {

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
        if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mCamera = mSceneManager->createCamera( "Main Camera" );

            // Position it at 500 in Z direction
            mCamera->setPosition( Ogre::Vector3( 0.0, 0.0, 0.0 ) );
            // Look back along -Z
            mCamera->lookAt( Ogre::Vector3( 0, 0, -1.0 ) );
            mCamera->setNearClipDistance( 0.2f );
            mCamera->setFarClipDistance( 20.0f );
            mCamera->setAutoAspectRatio( true );
            mCamera->detachFromParent();
            mCamerasNode->attachObject( mCamera );
        }

    }

    Ogre::CompositorWorkspace* StereoGraphicsSystem::setupCompositor()
    {
        Ogre::CompositorManager2 *compositorManager = mRoot->getCompositorManager2();
        const Ogre::IdString workspaceName( "TwoCamerasWorkspace" );

        if ( mWorkSpaceType == WS_TWO_CAMERAS_STEREO )
        {
            Ogre::uint8 vpModifierMask, executionMask;
            Ogre::Vector4 vpOffsetScale;

            vpModifierMask  = 0x01;
            executionMask   = 0x01;
            vpOffsetScale   = Ogre::Vector4( 0.0f, 0.0f, 0.5f, 1.0f );
            mEyeWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager,
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
            mEyeWorkspaces[LEFT] = compositorManager->addWorkspace(
                mSceneManager,
                mRenderWindow->getTexture(),
                mEyeCameras[RIGHT], workspaceName,
                true, -1, (Ogre::UavBufferPackedVec*)0,
                (Ogre::ResourceLayoutMap*)0,
                (Ogre::ResourceAccessMap*)0,
                vpOffsetScale,
                vpModifierMask,
                executionMask);
        }
        if (mWorkSpaceType == WS_INSTANCED_STEREO)
        {
            mEyeWorkspaces[0] = compositorManager->addWorkspace(
                mSceneManager, mRenderWindow->getTexture(), mCamera,
                "InstancedStereoWorkspace", true );

/*            mVrCullCamera = mSceneManager->createCamera( "VrCullCamera" );
            Ogre::CompositorWorkspace* vrCompositor = nullptr;
            vr::IVRCompositor *vrCompositor3D = nullptr;
//             Ogre::LogManager::getSingleton().logMessage(
//                 "setupCompositor");
            Ogre::CompositorManager2 *compositorManager =
                mRoot->getCompositorManager2();
            initOpenVR();

            Ogre::CompositorChannelVec channels( 2u );
            channels[0] = mRenderWindow->getTexture();
            channels[1] = mVrTexture;
            vrCompositor = compositorManager->addWorkspace(
                mSceneManager, channels, mCamera,
                "InstancedStereoMirrorWindowWorkspace", true );
            vrCompositor3D =
                vr::VRCompositor();
            int frames = compositorManager->getRenderSystem()->getVaoManager()->getDynamicBufferMultiplier();
            mOvrCompositorListener =
                new Demo::OpenVRCompositorListener(
                    mHMD, vrCompositor3D, mVrTexture,
                    mRoot, mVrWorkspace,
                    mCamera, mVrCullCamera,
                    frames );

            return vrCompositor*/;
        }

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
