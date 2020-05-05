
#ifndef _Demo_StereoRenderingVideoLoader_H_
#define _Demo_StereoRenderingVideoLoader_H_

#include "StereoRendering.h"
#include "StereoRenderingGraphicsSystem.h"

#include "GameState.h"
#include "Threading/MessageQueueSystem.h"


namespace Demo {
    class VideoLoader : public Mq::MessageQueueSystem
    {
        VideoInput mVideoInput;
        cv::VideoCapture mCapture;
        int captureFrameWidth;
        int captureFrameHeight;
        int captureFramePixelFormat;
        //TODO: cannot be included for some reason..
//         StereoGraphicsSystem     *mGraphicsSystem;

    public:
        VideoLoader(
//             StereoGraphicsSystem *graphicsSystem,
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
