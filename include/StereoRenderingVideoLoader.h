
#ifndef _Demo_StereoRenderingVideoLoader_H_
#define _Demo_StereoRenderingVideoLoader_H_

#include "StereoRendering.h"

#include "Threading/MessageQueueSystem.h"


namespace Demo {
    class StereoGraphicsSystem;

    class VideoLoader : public Mq::MessageQueueSystem
    {
        StereoGraphicsSystem *mGraphicsSystem;
        VideoInput mVideoInput;
        cv::VideoCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;
        int mCaptureFramePixelFormat;

    public:
        VideoLoader(
            StereoGraphicsSystem *graphicsSystem,
            VideoInput vInput );
        ~VideoLoader();

        void initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );

        void beginFrameParallel(void);
        void finishFrameParallel(void);
        void finishFrame(void);

        void processIncomingMessage( Mq::MessageId messageId, const void *data );
    };
}

#endif
