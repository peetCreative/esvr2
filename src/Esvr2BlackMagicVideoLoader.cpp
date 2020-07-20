#ifdef USE_BLACKMAGICCAMERA
#include "Esvr2BlackMagicVideoLoader.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"

#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
// #include <opencv2/core/cvstd.hpp>

#include "BlackMagicCapture.h"


namespace esvr2
{
    BlackMagicVideoLoader::BlackMagicVideoLoader(
            GraphicsSystem *graphicsSystem,
            VideoInput vInput):
        VideoLoader(graphicsSystem),
        mVideoInput( vInput ),
        mCapture(1, CBlackMagicCapture::eHD1080p50, false),
        mCaptureFrameWidth( 0 ),
        mCaptureFrameHeight( 0 ),
        mCaptureFramePixelFormat( 0 )
    {}

    BlackMagicVideoLoader::~BlackMagicVideoLoader() {}

    void BlackMagicVideoLoader::initialize(void)
    {
        if (!mCapture.OpenCamera()) {
            LOG << "Video could not be opened" << LOGEND;
            return;
        }
        // Default resolution of the frame is obtained.The default resolution is system dependent.
        mCaptureFrameWidth = mCapture.GetWidth();
        mCaptureFrameHeight = mCapture.GetHeight();
    }

    void BlackMagicVideoLoader::deinitialize(void)
    {
        mCapture.CloseCamera();
    }

    void BlackMagicVideoLoader::update( float timeSinceLast )
    {
        cv::Mat image(mCaptureFrameHeight, mCaptureFrameWidth, CV_8UC4);

        // Capture frame-by-frame
        mCapture.CaptureImage(&image); 
        if(image.empty())
        {
            return;
        }
        cv::Rect lrect, rrect;
        cv::Mat imageOrigLeft, imageOrigRight;
        cv::Mat *imageOrigLeftPtr = nullptr;
        cv::Mat *imageOrigRightPtr  = nullptr;
        switch ( mVideoInput.videoInputType )
        {
        case VIT_MONO:
            imageOrigLeftPtr = &image;
            break;
        case VIT_STEREO_SLICED:
        {
            size_t outputRows = mCaptureFrameHeight/2;

            if( mCaptureFrameHeight % 2 != 0 ) {
                LOG << "Height of input image must be divisible by 2 (but current height is " << mCaptureFrameHeight  << ")!";
                quit();
                return;
            }

            imageOrigLeft = cv::Mat( outputRows,  mCaptureFrameWidth, CV_8UC4 );
            imageOrigRight = cv::Mat( outputRows,  mCaptureFrameWidth, CV_8UC4 );

            // Split the input image into left and right, line by line.
            //TODO: normally First line is right image,
            //but somehow here it is different
            for( int inputRow = 0; inputRow < mCaptureFrameHeight; inputRow++ )
            {
                if( inputRow % 2 == 0 )
                {
                    int outputRow = inputRow/2;
                    unsigned int srcPos = inputRow * mCaptureFrameWidth * sizeof(unsigned char) *4;
                    unsigned int destPos = outputRow*mCaptureFrameWidth*sizeof(unsigned char)*4;
                    memcpy( imageOrigLeft.data + destPos, image.data + srcPos, sizeof(unsigned char)*4*mCaptureFrameWidth );
                }
                else
                {
                    int outputRow = (inputRow-1)/2;
                    unsigned int srcPos = inputRow*mCaptureFrameWidth*sizeof(unsigned char)*4;
                    unsigned int destPos = outputRow*mCaptureFrameWidth*sizeof(unsigned char)*4;
                    memcpy( imageOrigRight.data + destPos, image.data + srcPos, sizeof(unsigned char)*4*mCaptureFrameWidth );
                }
            }
            imageOrigLeftPtr = &imageOrigLeft;
            imageOrigRightPtr = &imageOrigRight;
        }
            break;
        case VIT_STEREO_VERTICAL_SPLIT:
        case VIT_STEREO_HORIZONTAL_SPLIT:
            LOG << "Not implemented!" << LOGEND;
            return;
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

    void BlackMagicVideoLoader::processIncomingMessage(
        Demo::Mq::MessageId messageId, const void *data )
    {}

}
#endif
