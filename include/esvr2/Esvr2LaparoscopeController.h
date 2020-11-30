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
        virtual bool moveLaparoscopeTo(
                LaparoscopeDOFPose);
        virtual LaparoscopeDOFPose getLaparoscopePose();
        virtual LaparoscopeDOFBoundaries getLaparoscopeBoundarySwingX();
    };
}

#endif //ESVR2_ESVR2LAPAROSCOPECONTROLLER_H
