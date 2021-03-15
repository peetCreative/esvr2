#include "Esvr2TestPose.h"

namespace esvr2{
    TestPose::TestPose():
        PoseState()
    {
        setPose(
            {0,0,-0.1},
            {1,0,0,0} );
        mValidPose = true;
    }

    TestPose::~TestPose() {}
}
