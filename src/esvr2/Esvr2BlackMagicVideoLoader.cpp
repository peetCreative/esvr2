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
            VideoInput vInput,
            StereoCameraConfig cameraConfig,
            Distortion distortion, bool stereo):
        VideoLoader( distortion, stereo ),
        mVideoInput( vInput ),
        mCapture( 1, CBlackMagicCapture::eHD1080p50, false ),
        mCaptureFrameWidth( 0 ),
        mCaptureFrameHeight( 0 )
    {
        mCameraConfig = cameraConfig;
    }

    BlackMagicVideoLoader::~BlackMagicVideoLoader() {}

    bool BlackMagicVideoLoader::initialize(void)
    {
        //TODO: more configuration for BlackMagicCapture could be available
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
        if (mStereo && mVideoInput.videoInputType == VIT_MONO)
        {
            return false;
        }
        updateDestinationSize(
            mCameraConfig.cfg[LEFT].width, mCameraConfig.cfg[LEFT].height, 4u,
            mCameraConfig.cfg[LEFT].width* mCameraConfig.cfg[LEFT].height* 4u );
        updateMaps();
        mReady = true;
        return true;
    }

    void BlackMagicVideoLoader::deinitialize(void)
    {
        mCapture.CloseCamera();
    }

    void BlackMagicVideoLoader::update( )
    {
        cv::Mat image(mCaptureFrameHeight, mCaptureFrameWidth, CV_8UC4);

        // Capture frame-by-frame
        mCapture.CaptureImage(&image); 
        if(image.empty())
        {
            return;
        }
        mSeq++;
        cv::Rect lrect, rrect;
        cv::Mat imageOrigLeft, imageOrigRight;
        switch ( mVideoInput.videoInputType )
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
