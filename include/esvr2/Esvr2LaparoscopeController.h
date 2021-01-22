//
// Created by peetcreative on 27.11.20.
//

#ifndef ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
#define ESVR2_ESVR2LAPAROSCOPECONTROLLER_H

#include "Esvr2Component.h"

namespace esvr2
{
    typedef struct {
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        float transZ = 0;
    } LaparoscopeDOFPose;

    typedef struct {
        float yawMax, yawMin;
        float pitchMax, pitchMin;
        float rollMax, rollMin;
        float transZMax, transZMin;
    } LaparoscopeDOFBoundaries;

    class LaparoscopeController : virtual public Component {
    protected:
        bool mLaparoscopeDofPoseReady = false;
        bool mLaparoscopeDofBoundariesReady = false;
    public:
        virtual bool moveLaparoscopeTo(
                LaparoscopeDOFPose) = 0;
        virtual bool getLaparoscopePose(
                LaparoscopeDOFPose &laparoscopeDofPose) = 0;
        virtual bool getLaparoscopeBoundaries(
                LaparoscopeDOFBoundaries &laparoscopeDofBoundaries) = 0;
        bool isReady() {
            return mLaparoscopeDofBoundariesReady && mLaparoscopeDofPoseReady;};
    };
}

#endif //ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
