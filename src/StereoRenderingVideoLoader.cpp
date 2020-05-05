#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

#include "opencv2/opencv.hpp"

// using namespace cv;

namespace Demo
{
    VideoLoader::VideoLoader(VideoInput vInput):
        mVideoInput(vInput),
        mCapture(),
        captureFrameWidth(0),
        captureFrameHeight(0),
        captureFramePixelFormat(0)

    {
        mVideoInput = vInput;
    }

    VideoLoader::~VideoLoader()
    {
    }

    void VideoLoader::initialize(void)
    {
        std::cout << mVideoInput.path << std::endl;
        if (mVideoInput.path == "")
        {
            LOG << "Video could not be opened" <<LOGEND;
            return;
        }
        mCapture = cv::VideoCapture(mVideoInput.path);
//         mVideoInput->capture.set(CV_CAP_PROP_MODE,  CV_CAP_MODE_RGB );
        if (!mCapture.isOpened()) {
            LOG << "Video could not be opened" <<LOGEND;
            return;
        }
        // Default resolution of the frame is obtained.The default resolution is system dependent.
// //         mVideoInput->captureFrameWidth =
// //             mVideoInput->capture.get(CV_CAP_PROP_FRAME_WIDTH);
// //         mVideoInput->captureFrameHeight =
// //             mVideoInput->capture.get(CV_CAP_PROP_FRAME_HEIGHT);
// //         mVideoInput->captureFramePixelFormat =
// //             mVideoInput->capture.get(CV_CAP_PROP_FORMAT);
    }

    void VideoLoader::deinitialize(void)
    {
    }

    //-----------------------------------------------------------------------------------
    void VideoLoader::beginFrameParallel(void)
    {
        this->processIncomingMessages();
    }

    void VideoLoader::update( float timeSinceLast )
    {
        cv::Mat mMat;

        // Capture frame-by-frame
        mCapture >> mMat; //1920/1080
        if(mMat.empty())
        {
//             Ogre::LogManager::getSingleton().logMessage("mMat empty");
            return;
        }
        if (mVideoInput.videoInputType == VIDEO_STEREO_VERTICAL_SPLIT)
        {
            cv::Rect lrect(0,540, 1920, 540);
            cv::Mat imageOrigLeft = mMat(lrect);
            cv::Rect rrect(0,0, 1920, 540);
            cv::Mat imageOrigRight = mMat(rrect);

            if( imageOrigLeft.empty() || imageOrigRight.empty() )
            {
                return;
            }
//             mGraphicsSystem->setImgPtr( &imageOrigLeft, &imageOrigRight );
        }
        else
        {
//             mGraphicsSystem->setImgPtr( &imageOrigLeft, &imageOrigRight );
        }
    }

    //-----------------------------------------------------------------------------------
    void VideoLoader::finishFrameParallel(void)
    {
        this->flushQueuedMessages();
    }

    //-----------------------------------------------------------------------------------
    void VideoLoader::finishFrame(void)
    {
    }

    void VideoLoader::processIncomingMessage( Mq::MessageId messageId, const void *data )
    {}

}
