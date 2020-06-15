#include "Esvr2VideoLoader.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2GraphicsSystem.h"

namespace esvr2
{
    VideoLoader::VideoLoader( GraphicsSystem *graphicsSystem ):
        mGraphicsSystem( graphicsSystem ),
        mQuit(false)
    {}

    VideoLoader::~VideoLoader()
    {}

    void VideoLoader::initialize(void) {}
    void VideoLoader::deinitialize(void) {}
    void VideoLoader::update( float timeSinceLast ) {}

    //-----------------------------------------------------------------------------------
    void VideoLoader::beginFrameParallel(void)
    {
        this->processIncomingMessages();
    }

    //-----------------------------------------------------------------------------------
    void VideoLoader::finishFrameParallel(void)
    {
        this->flushQueuedMessages();
    }

    //-----------------------------------------------------------------------------------
    void VideoLoader::finishFrame(void) {}

    void VideoLoader::processIncomingMessage(
        Demo::Mq::MessageId messageId, const void *data ) {}
}
