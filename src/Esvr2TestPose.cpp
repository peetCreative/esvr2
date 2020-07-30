#include "Esvr2TestPose.h"

namespace esvr2{
    TestPose::TestPose():
        PoseState()
    {
//         Ogre::Vector3 position( 0, 0, 1 );
        Ogre::Vector3 position( -0.00065564, 0.0158165, -0.136581 );
        Ogre::Quaternion orientation(  -0.238682, 0.0333106, 0.970032, -0.0309802 ); 
//         Ogre::Quaternion orientation( -0.96333468, 0.26830256, 0,0); 
//         Ogre::Quaternion orientation( Ogre::Radian(0.174533), 
//                                       Ogre::Vector3(1, 1, 0 ));
        setPose( position, orientation );
        mValidPose = true;
    }

    TestPose::~TestPose() {}
}
