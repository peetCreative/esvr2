
#ifndef _Demo_StereoRenderingOpenCvVideoLoader_H_
#define _Demo_StereoRenderingOpenCvVideoLoader_H_

#include "StereoRendering.h"

#include "StereoRenderingVideoLoader.h"

#include "opencv2/opencv.hpp"


namespace Demo {
    class StereoGraphicsSystem;

    class OpenCvVideoLoader : public VideoLoader
    {
        VideoInput mVideoInput;
        cv::VideoCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;
        int mCaptureFramePixelFormat;

    public:
        OpenCvVideoLoader(
            StereoGraphicsSystem *graphicsSystem,
            VideoInput vInput );
        ~OpenCvVideoLoader();

        void initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );

        void processIncomingMessage( Mq::MessageId messageId, const void *data );
    };
}

#endif
