#include "StereoRenderingOpenCvVideoLoader.h"
#include "StereoRenderingVideoLoader.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

#include "opencv2/opencv.hpp"

namespace esvr2
{
    OpenCvVideoLoader::OpenCvVideoLoader(
            StereoGraphicsSystem *graphicsSystem,
            VideoInput vInput):
        VideoLoader(graphicsSystem),
        mVideoInput( vInput ),
        mCapture(),
        mCaptureFrameWidth( 0 ),
        mCaptureFrameHeight( 0 ),
        mCaptureFramePixelFormat( 0 )
    {}

    OpenCvVideoLoader::~OpenCvVideoLoader() {}

    void OpenCvVideoLoader::initialize(void)
    {
        std::cout << mVideoInput.path << std::endl;
        if (mVideoInput.path == "")
        {
            LOG << "Video could not be opened" << LOGEND;
            return;
        }
        mCapture = cv::VideoCapture(mVideoInput.path);
//         mCapture.set(cv::CAP_PROP_MODE,  cv::CAP_MODE_RGB );
        if (!mCapture.isOpened()) {
            LOG << "Video could not be opened" << LOGEND;
            return;
        }
        // Default resolution of the frame is obtained.The default resolution is system dependent.
        mCaptureFrameWidth =
            mCapture.get(cv::CAP_PROP_FRAME_WIDTH);
        mCaptureFrameHeight =
            mCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
    }

    void OpenCvVideoLoader::deinitialize(void)
    {
        mCapture.release();
    }

    void OpenCvVideoLoader::update( float timeSinceLast )
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
            lrect = cv::Rect(0, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            rrect = cv::Rect( mCaptureFrameWidth/2, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            imageOrigLeft = mMat(lrect);
            imageOrigLeftPtr = &imageOrigLeft;
            imageOrigRight = mMat(rrect);
            imageOrigRightPtr = &imageOrigRight;
            break;
        case VIDEO_STEREO_HORIZONTAL_SPLIT:
            // left is below
            // right is above
            lrect = cv::Rect(0, mCaptureFrameHeight/2,
                           mCaptureFrameWidth, mCaptureFrameHeight/2);
            rrect = cv::Rect(0, 0,
                           mCaptureFrameWidth, mCaptureFrameHeight/2);
            imageOrigLeft = mMat(lrect);
            imageOrigLeftPtr = &imageOrigLeft;
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

    void OpenCvVideoLoader::processIncomingMessage(
        Demo::Mq::MessageId messageId, const void *data )
    {}

}
