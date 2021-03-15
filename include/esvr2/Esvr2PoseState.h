#ifndef _Esvr2_POSESTATE_H_
#define _Esvr2_POSESTATE_H_

#include "Esvr2.h"
#include "Esvr2Component.h"

//Class to externally control the movements
// of the laparoscope camera in our application
// used for debugging

namespace esvr2 {
    class PoseState: virtual public Component {
    protected:
        RealArray16 mPose;

        RealArray3 mPosition;
        RealArray4 mOrientation;
        void setPose(RealArray3 position, RealArray4 orientation);

        bool mValidPose;

    public:
        PoseState();
        ~PoseState();

        virtual RealArray16 getPose(void);
        //relativ to parent
        virtual RealArray3 getPosition(void);
        virtual RealArray4 getOrientation(void);
        bool validPose();

    };
    typedef std::shared_ptr<PoseState> PoseStatePtr;
}

#endif
