#ifndef _Esvr2_POSESTATE_H_
#define _Esvr2_POSESTATE_H_

#include "Esvr2StereoRendering.h"

namespace esvr2 {
    class PoseState {
    protected:
        Ogre::Matrix4 mPose;

        Ogre::Vector3 mPosition;
        Ogre::Quaternion mRotation;
        void setPose(Ogre::Vector3 position, Ogre::Quaternion rotation);

        bool mValidPose;

    public:
        PoseState();
        ~PoseState();

        Ogre::Matrix4 getPose(void);
        //relativ to parent
        Ogre::Vector3 getPosition(void);
        Ogre::Quaternion getRotation(void);
        bool validPose();

    };
}

#endif
