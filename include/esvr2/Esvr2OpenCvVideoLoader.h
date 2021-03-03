
#ifndef _Esvr2_OpenCvVideoLoader_H_
#define _Esvr2_OpenCvVideoLoader_H_

#include "Esvr2.h"

#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"


namespace esvr2 {
    class GraphicsSystem;

    class OpenCvVideoLoader : public VideoLoader
    {
        std::string mPath;
        VideoInputType mVideoInputType;
        cv::VideoCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;

    public:
        OpenCvVideoLoader(
                const VideoInputConfigPtr videoInputConfig);
        ~OpenCvVideoLoader();

        bool initialize(void);
        void deinitialize(void);
        void update( );

    };
}

#endif
