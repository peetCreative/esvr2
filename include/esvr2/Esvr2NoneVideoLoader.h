#pragma once
#include "Esvr2.h"
#include "Esvr2VideoLoader.h"

namespace esvr2
{
    class NoneVideoLoader: virtual public VideoLoader
    {
    public:
        NoneVideoLoader():
        VideoLoader(DIST_RAW, true)
        {
            mCameraConfig.leftCameraConfig.width = 400;
            mCameraConfig.leftCameraConfig.height = 400;
            mCameraConfig.rightCameraConfig.width = 400;
            mCameraConfig.rightCameraConfig.height = 400;
            mReady = true;
        };
    };
}