
#ifndef _Esvr2_VideoLoader_H_
#define _Esvr2_VideoLoader_H_

#include "Esvr2StereoRendering.h"

#include "Threading/MessageQueueSystem.h"

namespace esvr2 {
    class GraphicsSystem;

    class VideoLoader : public Demo::Mq::MessageQueueSystem
    {
    protected:
        GraphicsSystem *mGraphicsSystem;
        bool mQuit;

    public:
        VideoLoader( GraphicsSystem *graphicsSystem );
        ~VideoLoader();

        virtual void initialize( void );
        virtual void deinitialize(void);
        virtual void update( float timeSinceLast );

        void beginFrameParallel(void);
        void finishFrameParallel(void);
        void finishFrame(void);

        virtual void processIncomingMessage( Demo::Mq::MessageId messageId, const void *data );

        void quit() {mQuit = true;};
        bool getQuit() {return mQuit;};
    };
}

#endif
