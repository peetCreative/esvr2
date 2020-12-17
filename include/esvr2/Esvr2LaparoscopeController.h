//
// Created by peetcreative on 27.11.20.
//

#ifndef ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
#define ESVR2_ESVR2LAPAROSCOPECONTROLLER_H

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

    class LaparoscopeController {
    public:
        virtual bool moveLaparoscopeTo(
                LaparoscopeDOFPose) = 0;
        virtual bool getLaparoscopePose(
                LaparoscopeDOFPose &laparoscopeDofPose) = 0;
        virtual bool getLaparoscopeBoundaries(
                LaparoscopeDOFBoundaries &laparoscopeDofBoundaries) = 0;
    };
}

#endif //ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
