
#ifndef _Esvr2_OpenCvVideoLoader_H_
#define _Esvr2_OpenCvVideoLoader_H_

#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"


namespace esvr2 {
    class GraphicsSystem;

    class OpenCvVideoLoader : public VideoLoader
    {
        VideoInput mVideoInput;
        cv::VideoCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;

    public:
        OpenCvVideoLoader(
            GraphicsSystem *graphicsSystem,
            VideoInput vInput );
        ~OpenCvVideoLoader();

        bool initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );

    };
}

#endif
