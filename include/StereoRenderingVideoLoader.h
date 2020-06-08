
#ifndef _Demo_StereoRenderingVideoLoader_H_
#define _Demo_StereoRenderingVideoLoader_H_

#include "StereoRendering.h"

#include "Threading/MessageQueueSystem.h"

namespace esvr2 {
    class StereoGraphicsSystem;

    class VideoLoader : public Demo::Mq::MessageQueueSystem
    {
    protected:
        StereoGraphicsSystem *mGraphicsSystem;

    public:
        VideoLoader( StereoGraphicsSystem *graphicsSystem );
        ~VideoLoader();

        virtual void initialize( void );
        virtual void deinitialize(void);
        virtual void update( float timeSinceLast );

        void beginFrameParallel(void);
        void finishFrameParallel(void);
        void finishFrame(void);

        virtual void processIncomingMessage( Demo::Mq::MessageId messageId, const void *data );

        virtual bool getQuit() {return false;};
    };
}

#endif
