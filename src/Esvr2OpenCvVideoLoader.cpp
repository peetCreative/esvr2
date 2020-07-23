#include "Esvr2OpenCvVideoLoader.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"

#include "opencv2/opencv.hpp"

namespace esvr2
{
    OpenCvVideoLoader::OpenCvVideoLoader(
            VideoInput vInput,
            StereoCameraConfig cameraConfig,
            Distortion distortion, bool stereo):
        VideoLoader( distortion, stereo ),
        mVideoInput( vInput ),
        mCapture(),
        mCaptureFrameWidth( 0 ), mCaptureFrameHeight( 0 )
    {
        mCameraConfig = cameraConfig;
    }

    OpenCvVideoLoader::~OpenCvVideoLoader() {}

    bool OpenCvVideoLoader::initialize(void)
    {
        LOG << "Opening video: " << mVideoInput.path << LOGEND;
        if (mVideoInput.path == "")
        {
            LOG << "Video could not be opened" << LOGEND;
            return false;
        }

        mCapture = cv::VideoCapture();
        cv::String path = mVideoInput.path;
        mCapture.open( path );
//         mCapture.set(cv::CAP_PROP_MODE,  cv::CAP_MODE_RGB );
        if (!mCapture.isOpened()) {
            LOG << "Video could not be opened" << LOGEND;
            return false;
        }

        //we cannot be stereo if we only capture mono
        if (mStereo && mVideoInput.videoInputType == VIT_MONO)
            return false;
        updateDestinationSize(
            mCameraConfig.cfg[LEFT].width, mCameraConfig.cfg[LEFT].height, 4u,
            mCameraConfig.cfg[LEFT].width* mCameraConfig.cfg[LEFT].height* 4u );
        updateMaps();
        mReady = true;
        return true;
    }

    void OpenCvVideoLoader::deinitialize(void)
    {
        mCapture.release();
    }

    void OpenCvVideoLoader::update( )
    {
        cv::Mat mMat;

        // Capture frame-by-frame
        mCapture >> mMat; //1920/1080
        if(mMat.empty())
        {
//             Ogre::LogManager::getSingleton().logMessage("mMat empty");
            return;
        }
        mSeq++;
        cv::Rect lrect, rrect;
        cv::Mat imageOrigLeft, imageOrigRight;
        switch ( mVideoInput.videoInputType )
        {
        case VIT_MONO:
            setImageDataFromRaw(&mMat, nullptr);
            break;
        case VIT_STEREO_SLICED:
            setImageDataFromSplitSliced(&mMat);
            break;
        case VIT_STEREO_VERTICAL_SPLIT:
            setImageDataFromSplit(&mMat, ORIENTATION_VERTICAL);
            break;
        case VIT_STEREO_HORIZONTAL_SPLIT:
            setImageDataFromSplit(&mMat, ORIENTATION_HORIZONTAL);
            break;
        default:
            break;
        }
    }
}
