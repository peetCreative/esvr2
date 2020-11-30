#include "Esvr2TestPose.h"

namespace esvr2{
    TestPose::TestPose():
        PoseState()
    {
        Ogre::Vector3 position( 0, 0, -0.1 );
        Ogre::Quaternion orientation(1,0,0,0);
        setPose( position, orientation );
        mValidPose = true;
    }

    TestPose::~TestPose() {}
}
