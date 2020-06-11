#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

namespace esvr2
{
    VideoLoader::VideoLoader( StereoGraphicsSystem *graphicsSystem ):
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
