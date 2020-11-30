#include "Esvr2PoseState.h"

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreMatrix4.h>
namespace esvr2 {
    PoseState::PoseState():
        mPose( Ogre::Matrix4::IDENTITY ),
        mOrientation (Ogre::Quaternion::IDENTITY),
        mPosition (Ogre::Vector3::ZERO),
        mValidPose(false){}

    PoseState::~PoseState(){}

    Ogre::Matrix4 PoseState::getPose()
    {
        return mPose;
    }

    Ogre::Vector3 PoseState::getPosition()
    {
        return mPosition;
    }

    Ogre::Quaternion PoseState::getOrientation()
    {
        return mOrientation;
    }

    void PoseState::setPose(
        Ogre::Vector3 position, Ogre::Quaternion orientation )
    {
        mPosition = position;
        mOrientation = orientation;
        mPose = Ogre::Matrix4(orientation);
        mPose.setTrans(position);
        mValidPose = true;
    }

    bool PoseState::validPose()
    {
        return mValidPose;
    }

};
