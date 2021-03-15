#include "Esvr2PoseState.h"
#include "Esvr2Helper.h"
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreMatrix4.h>
namespace esvr2 {
    PoseState::PoseState():
        mPose(Matrix4ToRealArray16(Ogre::Matrix4::IDENTITY)),
        mOrientation (QuaternionToRealArray4(Ogre::Quaternion::IDENTITY)),
        mPosition (Vector3ToRealArray3(Ogre::Vector3::ZERO)),
        mValidPose(false){}

    PoseState::~PoseState(){}

    RealArray16 PoseState::getPose()
    {
        return mPose;
    }

    RealArray3 PoseState::getPosition()
    {
        return mPosition;
    }

    RealArray4 PoseState::getOrientation()
    {
        return mOrientation;
    }

    void PoseState::setPose(
        RealArray3 position, RealArray4 orientation )
    {
        mPosition = position;
        mOrientation = orientation;
        Ogre::Matrix4 pose = Ogre::Matrix4(RealArray4ToQuaternion(orientation));
        pose.setTrans(RealArray3ToVector3(position));
        mPose = Matrix4ToRealArray16(pose);
        mValidPose = true;
    }

    bool PoseState::validPose()
    {
        return mValidPose;
    }

};
