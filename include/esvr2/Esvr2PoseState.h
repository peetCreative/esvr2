#ifndef _Esvr2_POSESTATE_H_
#define _Esvr2_POSESTATE_H_

#include "Esvr2.h"

//Class to externally control the movements of the camera in our application
// used for debugging

namespace esvr2 {
    class PoseState {
    protected:
        Ogre::Matrix4 mPose;

        Ogre::Vector3 mPosition;
        Ogre::Quaternion mOrientation;
        void setPose(Ogre::Vector3 position, Ogre::Quaternion orientation);

        bool mValidPose;

    public:
        PoseState();
        ~PoseState();

        Ogre::Matrix4 getPose(void);
        //relativ to parent
        Ogre::Vector3 getPosition(void);
        Ogre::Quaternion getOrientation(void);
        bool validPose();

    };
}

#endif
