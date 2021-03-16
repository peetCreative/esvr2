#include "Esvr2OpenCvVideoLoader.h"

#include "Esvr2.h"

#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"

namespace esvr2
{
    OpenCvVideoLoader::OpenCvVideoLoader(
            const VideoInputConfigPtr videoInputConfig):
        VideoLoader( videoInputConfig->distortion, videoInputConfig->isStereo ),
        mPath( videoInputConfig->path ),
        mVideoInputType(videoInputConfig->videoInputType),
        mCapture(),
        mCaptureFrameWidth( 0 ), mCaptureFrameHeight( 0 )
    {
        mCameraConfig = videoInputConfig->stereoCameraConfig;
    }

    OpenCvVideoLoader::~OpenCvVideoLoader() = default;

    bool OpenCvVideoLoader::initialize(void)
    {
        LOG << "Opening video: " << mPath << LOGEND;
        if (mPath == "")
        {
            LOG << "Video could not be opened" << LOGEND;
            return false;
        }

        mCapture = cv::VideoCapture();
        cv::String path = mPath;
        mCapture.open( path);
        mCapture.set(cv::CAP_PROP_CONVERT_RGB,  true );
        if (!mCapture.isOpened()) {
            LOG << "Video could not be opened" << LOGEND;
            return false;
        }

        //we cannot be stereo if we only capture mono
        if (mStereo && mVideoInputType == VIT_MONO)
            return false;
        updateDestinationSize(
            mCameraConfig.cfg[LEFT]->width, mCameraConfig.cfg[LEFT]->height, 4u,
            mCameraConfig.cfg[LEFT]->width* mCameraConfig.cfg[LEFT]->height* 4u );
        updateMaps();
        mColorConversion = cv::COLOR_RGB2BGRA;
        mReady = true;
        return true;
    }

    void OpenCvVideoLoader::deinitialize(void)
    {
        mCapture.release();
    }

    void OpenCvVideoLoader::update(uint64 time)
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
        switch ( mVideoInputType )
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
