#include "Esvr2PoseState.h"

#include "Esvr2StereoRendering.h"
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreMatrix4.h>
namespace esvr2 {
    PoseState::PoseState():
        mPose( Ogre::Matrix4::IDENTITY ),
        mRotation (Ogre::Quaternion::IDENTITY),
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

    Ogre::Quaternion PoseState::getRotation()
    {
        return mRotation;
    }

    void PoseState::setPose(
        Ogre::Vector3 position, Ogre::Quaternion rotation )
    {
        mPosition = position;
        mRotation = rotation;
        mPose = Ogre::Matrix4(rotation);
        mPose.setTrans(position);
    }

    bool PoseState::validPose()
    {
        return mValidPose;
    }

};
