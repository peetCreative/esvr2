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
}
