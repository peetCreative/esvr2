#ifdef USE_BLACKMAGICCAMERA
#include "Esvr2BlackMagicVideoLoader.h"

#include "Esvr2.h"

#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"

#include "opencv2/opencv.hpp"

#include "BlackMagicCapture.h"


namespace esvr2
{
    BlackMagicVideoLoader::BlackMagicVideoLoader(
            VideoInputConfigPtr videoInputConfig):
        VideoLoader( videoInputConfig->distortion, videoInputConfig->isStereo ),
        mVideoInputConfig(videoInputConfig),
        mCapture( 1, CBlackMagicCapture::eHD1080p50, false ),
        mCaptureFrameWidth( 0 ),
        mCaptureFrameHeight( 0 )
    {
        mCameraConfig = videoInputConfig->stereoCameraConfig;
    }

    BlackMagicVideoLoader::~BlackMagicVideoLoader() {}

    //! TODO: more configuration for BlackMagicCapture could be available
    bool BlackMagicVideoLoader::initialize(void)
    {
        if (!mCapture.OpenCamera()) {
            LOG << "Video could not be opened" << LOGEND;
            return false;
        }
        // Default resolution of the frame is obtained.The default resolution is system dependent.
        mCaptureFrameWidth = mCapture.GetWidth();
        mCaptureFrameHeight = mCapture.GetHeight();

        if ( mCaptureFrameWidth == 0 || mCaptureFrameHeight == 0 )
        {
            return false;
        }
        if (mStereo && mVideoInputConfig->videoInputType == VIT_MONO)
        {
            return false;
        }
        updateDestinationSize(
            mCameraConfig.cfg[LEFT]->width, mCameraConfig.cfg[LEFT]->height, 4u,
            mCameraConfig.cfg[LEFT]->width* mCameraConfig.cfg[LEFT]->height* 4u );
        updateMaps();
        mColorConversion = cv::COLOR_RGBA2BGRA;
        mReady = true;
        return true;
    }

    void BlackMagicVideoLoader::deinitialize(void)
    {
        mCapture.CloseCamera();
    }

    /*! takes the image from Capture Object and converts it to openCV
     * the rest are the functions from the superclass VideoLoader doing
     */
    void BlackMagicVideoLoader::update( uint64 time )
    {
        cv::Mat image(mCaptureFrameHeight, mCaptureFrameWidth, CV_8UC4);

        // Capture frame-by-frame
        mCapture.CaptureImage({&image});
        if(image.empty())
        {
            return;
        }
        mSeq++;
        cv::Rect lrect, rrect;
        cv::Mat imageOrigLeft, imageOrigRight;
        switch ( mVideoInputConfig->videoInputType )
        {
            case VIT_MONO:
                setImageDataFromRaw(&image, nullptr);
                break;
            case VIT_STEREO_SLICED:
                setImageDataFromSplitSliced(&image);
                break;
            case VIT_STEREO_VERTICAL_SPLIT:
                setImageDataFromSplit(&image, ORIENTATION_VERTICAL);
                break;
            case VIT_STEREO_HORIZONTAL_SPLIT:
                setImageDataFromSplit(&image, ORIENTATION_HORIZONTAL);
                break;
            default:
                break;
        }
    }
}
#endif
