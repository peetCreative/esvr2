#include "Esvr2OpenCvVideoLoader.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"

// #include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/core/cvstd.hpp>

namespace esvr2
{
    OpenCvVideoLoader::OpenCvVideoLoader(
            GraphicsSystem *graphicsSystem,
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

        mCapture = cv::VideoCapture();
        cv::String path = mVideoInput.path;
#ifndef USE_ROS
        mCapture.open( path );
//         mCapture.set(cv::CAP_PROP_MODE,  cv::CAP_MODE_RGB );
#endif
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
        case VIT_MONO:
            imageOrigLeftPtr = &mMat;
            break;
        case VIT_STEREO_SLICED:
        {
            size_t outputRows = mCaptureFrameHeight/2;

            if( mCaptureFrameHeight % 2 != 0 ) {
                LOG << "Height of input image must be divisible by 2 (but current height is " << mCaptureFrameHeight  << ")!";
                quit();
                return;
            }

            imageOrigLeft = cv::Mat( outputRows,  mCaptureFrameWidth, CV_8UC3 );
            imageOrigRight = cv::Mat( outputRows,  mCaptureFrameWidth, CV_8UC3 );

            // Split the input image into left and right, line by line.
            //TODO: normally First line is right image,
            //but somehow here it is different
            for( int inputRow = 0; inputRow < mCaptureFrameHeight; inputRow++ )
            {
                if( inputRow % 2 == 0 )
                {
                    int outputRow = inputRow/2;
                    unsigned int srcPos = inputRow * mCaptureFrameWidth * sizeof(unsigned char) *3;
                    unsigned int destPos = outputRow*mCaptureFrameWidth*sizeof(unsigned char)*3;
                    memcpy( imageOrigLeft.data + destPos, mMat.data + srcPos, sizeof(unsigned char)*3*mCaptureFrameWidth );
                }
                else
                {
                    int outputRow = (inputRow-1)/2;
                    unsigned int srcPos = inputRow*mCaptureFrameWidth*sizeof(unsigned char)*3;
                    unsigned int destPos = outputRow*mCaptureFrameWidth*sizeof(unsigned char)*3;
                    memcpy( imageOrigRight.data + destPos, mMat.data + srcPos, sizeof(unsigned char)*3*mCaptureFrameWidth );
                }
            }
            imageOrigLeftPtr = &imageOrigLeft;
            imageOrigRightPtr = &imageOrigRight;
        }
            break;
        case VIT_STEREO_VERTICAL_SPLIT:
            lrect = cv::Rect(0, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            rrect = cv::Rect( mCaptureFrameWidth/2, 0,
                           mCaptureFrameWidth/2, mCaptureFrameHeight);
            imageOrigLeft = mMat(lrect);
            imageOrigLeftPtr = &imageOrigLeft;
            imageOrigRight = mMat(rrect);
            imageOrigRightPtr = &imageOrigRight;
            break;
        case VIT_STEREO_HORIZONTAL_SPLIT:
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
