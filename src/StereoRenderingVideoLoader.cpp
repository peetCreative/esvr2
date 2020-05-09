#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

namespace Demo
{
    VideoLoader::VideoLoader( StereoGraphicsSystem *graphicsSystem ):
        mGraphicsSystem( graphicsSystem )
    {}

    VideoLoader::~VideoLoader()
    {
    }

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
        Mq::MessageId messageId, const void *data ) {}
}
