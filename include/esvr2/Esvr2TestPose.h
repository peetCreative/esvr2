#ifndef _Esvr2_TESTPOSE_H_
#define _Esvr2_TESTPOSE_H_

#include "Esvr2PoseState.h"

namespace esvr2 {
    //! \brief a test class to provide a dummy implementation for PoseState
    class TestPose : public PoseState
    {
    public:
        TestPose();
        ~TestPose();
    };
}

#endif
