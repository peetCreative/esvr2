//
// Created by peetcreative on 27.11.20.
//

#ifndef ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
#define ESVR2_ESVR2LAPAROSCOPECONTROLLER_H

#include "Esvr2Component.h"

namespace esvr2
{
    typedef struct {
        float swingX;
        float swingY;
        float transZ;
        float rotZ;
    } LaparoscopeDOFPose;

    typedef struct {
        float swingXMax, swingXMin;
        float swingYMax, swingYMin;
        float transZMax, transZMin;
        float rotZMax, rotZMin;
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
