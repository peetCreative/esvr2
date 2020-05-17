
#ifndef _Demo_StereoRenderingGameState_H_
#define _Demo_StereoRenderingGameState_H_

#include "OgrePrerequisites.h"
#include "TutorialGameState.h"

#include "OgreHlmsUnlitDatablock.h"
#include "OgreManualObject2.h"

namespace Demo
{
    class StereoRenderingGameState : public TutorialGameState
    {
        Ogre::SceneNode          *mSceneNodeCube;
        Ogre::HlmsUnlitDatablock *mVideoDatablock[2];
        Ogre::ManualObject       *mProjectionRectangle[2];
        Ogre::SceneNode          *mSceneNodeCamera;
        Ogre::SceneNode          *mSceneNodeLight;
        bool                     mIsStereo;
        size_t                   mEyeNum;

    public:
        StereoRenderingGameState(
            const Ogre::String &helpDescription,
            bool isStereo );
        void createCube();
        virtual void createScene01(void);

        virtual void update( float timeSinceLast );
    };
}

#endif
