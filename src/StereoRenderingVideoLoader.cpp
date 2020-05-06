#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

#include "opencv2/opencv.hpp"

// using namespace cv;

namespace Demo
{
    VideoLoader::VideoLoader(
            StereoGraphicsSystem *graphicsSystem,
            VideoInput vInput):
        mGraphicsSystem( graphicsSystem ),
        mVideoInput( vInput ),
        mCapture(),
        mCaptureFrameWidth( 0 ),
        mCaptureFrameHeight( 0 ),
        mCaptureFramePixelFormat( 0 )
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
        mCaptureFrameWidth =
            mCapture.get(cv::CAP_PROP_FRAME_WIDTH);
        mCaptureFrameHeight =
            mCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
//         mVideoInput->captureFramePixelFormat =
//             mVideoInput->capture.get(CV_CAP_PROP_FORMAT);
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
        cv::Rect lrect, rrect;
        cv::Mat imageOrigLeft, imageOrigRight;
        cv::Mat *imageOrigLeftPtr = nullptr;
        cv::Mat *imageOrigRightPtr  = nullptr;
        switch ( mVideoInput.videoInputType )
        {
        case VIDEO_MONO:
            imageOrigLeftPtr = &mMat;
            break;
        case VIDEO_STEREO_SLICED:
//             TODO:implement manage sliced images
            break;
        case VIDEO_STEREO_VERTICAL_SPLIT:
            //left is below
            lrect = cv::Rect(0, mCaptureFrameHeight/2,
                           mCaptureFrameWidth, mCaptureFrameHeight/2);
            imageOrigLeft = mMat(lrect);
            imageOrigLeftPtr = &imageOrigLeft;
            //right is above
            rrect = cv::Rect(0, 0,
                           mCaptureFrameWidth, mCaptureFrameHeight/2);
            imageOrigRight = mMat(rrect);
            imageOrigRightPtr = &imageOrigRight;
            break;
        case VIDEO_STEREO_HORIZONTAL_SPLIT:
            lrect = cv::Rect(0, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            imageOrigLeft = mMat(lrect);
            imageOrigLeftPtr = &imageOrigLeft;
            lrect = cv::Rect(mCaptureFrameWidth/2, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            imageOrigRight = mMat(rrect);
            imageOrigRightPtr = &imageOrigRight;
            break;
        default:
            break;
        }
        //check left Ptr is valid at least.
        //if right is nullptr graphics will project left to both eyes
        if( !imageOrigLeftPtr || imageOrigLeftPtr->empty() ||
            (imageOrigRightPtr && imageOrigRightPtr->empty()))
            return;
        mGraphicsSystem->setImgPtr( imageOrigLeftPtr, imageOrigRightPtr );
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
