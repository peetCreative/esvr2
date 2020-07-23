#include "Esvr2LowLatencyVideoLoader.h"
#include "Esvr2GraphicsSystem.h"

#include <fstream>

#include <linux/videodev2.h>

#include <opencv2/imgproc.hpp>

namespace esvr2 {

LowLatencyVideoLoader::LowLatencyVideoLoader(
        VideoInput videoInput, bool profilingEnabled,
        StereoCameraConfig cameraConfig,
        Distortion distortion):
    VideoLoader(distortion, true),
    mVideoInput(videoInput),
    mProfilingEnabled(profilingEnabled)
{
    mCameraConfig = cameraConfig;
}

LowLatencyVideoLoader::~LowLatencyVideoLoader()
{
}

// A helper function that reads the cycle count of the CPU. It is used
// for profiling to measure processor time.
inline uint64_t rdtsc() {
    uint32_t lo, hi;
    __asm__ __volatile__ (
      "xorl %%eax, %%eax\n"
      "cpuid\n"
      "rdtsc\n"
      : "=a" (lo), "=d" (hi)
      :
      : "%ebx", "%ecx");
    return (uint64_t)hi << 32 | lo;
}

bool LowLatencyVideoLoader::initialize()
{
    // open video interface
    try {
        //resolution and framerate are configured automatically
        videoInterface.open(mVideoInput.path, {0, 0}, {0, 0});
    } catch(v4l2::V4L2Interface::IOError &e) {
        LOG << "V4L2Interface::IOError: " <<  e.what() << LOGEND;
        quit();
        return false;
    }

    if(mProfilingEnabled) {
        mProfilingLog.open("profiling/camera_log.csv");
        mProfilingLog << "sequence,read_time,copy_time,send_time" << std::endl;
    }

    unsigned int inputRows       = videoInterface.getHeight(),
                cols            = videoInterface.getWidth(),
                outputRows      = inputRows / 2;

    if ( cols == 0 || outputRows == 0 )
    {
        return false;
    }
    mImageLeft = cv::Mat(cv::Size(cols, outputRows), CV_8UC3, cv::Scalar(0,0,0));
    mImageRight = cv::Mat(cv::Size(cols, outputRows), CV_8UC3, cv::Scalar(0,0,0));

    updateDestinationSize(
        mCameraConfig.cfg[LEFT].width, mCameraConfig.cfg[LEFT].height, 4u,
        mCameraConfig.cfg[LEFT].width* mCameraConfig.cfg[LEFT].height* 4u );
    updateMaps();
    mReady = true;
    return true;
}

void LowLatencyVideoLoader::deinitialize()
{
    videoInterface.close();
}

enum cv::ColorConversionCodes getColorConversion(const uint32_t &pixel_format) {
    // translate from v4l2 YUV pixel format names to the OpenCV color conversion
    // code for YUV to RGB conversion
    switch(pixel_format) {
        case V4L2_PIX_FMT_YUYV:
            return cv::COLOR_YUV2RGB_YUYV;
        case V4L2_PIX_FMT_UYVY:
            return cv::COLOR_YUV2RGB_UYVY;
        case V4L2_PIX_FMT_YVU420:
            return cv::COLOR_YUV2RGB_YV12;
        case V4L2_PIX_FMT_YUV420:
            return cv::COLOR_YUV2RGB_IYUV;
        case V4L2_PIX_FMT_NV12:
            return cv::COLOR_YUV2RGB_NV12;
        case V4L2_PIX_FMT_NV21:
            return cv::COLOR_YUV2RGB_NV21;
        default:
            return cv::COLOR_COLORCVT_MAX; // no conversion
    }
}


void LowLatencyVideoLoader::update( ) {
    mSeq++;
    unsigned int inputRows       = videoInterface.getHeight(),
                 cols            = videoInterface.getWidth(),
                 outputRows      = inputRows / 2,
                 rowLength       = cols * videoInterface.getBytesPerPixel(),
                 minBufferLength = inputRows * rowLength;
    v4l2::V4L2Interface::Buffer buffer;

    cv::Mat conversionMatrix;

    enum cv::ColorConversionCodes conversion = getColorConversion(videoInterface.getColorFormat());

    int opencv_convert_type;

    // For YUV to RGB conversion the number of bytes per row and rows per
    // image meight change. Adjusting the values accordingly.
    if(conversion != cv::COLOR_COLORCVT_MAX) {
        // convert 2-byte YUV to 3-byte RGB
        rowLength = rowLength * 3 / 2;
        opencv_convert_type = CV_8UC2;
        if(conversion != cv::COLOR_YUV2RGB_YUYV && conversion != cv::COLOR_YUV2RGB_UYVY) {
            // expand 2 rows YUV to 3 rows RGB
            inputRows = inputRows * 3 / 2;
            outputRows = inputRows / 2;
            opencv_convert_type = CV_8U;
        }
    }


    if(inputRows % 2 != 0) {
        LOG << "Height of input image is not divisible by 2 (but current height is " << inputRows << "), doing nothing!" << LOGEND;
        return;
    }

    buffer.start = NULL;
    buffer.length = 0;

    read_time = rdtsc();

    // read the image from the camera
    try {
        buffer = videoInterface.getImage();

        if(buffer.length < minBufferLength) {
            LOG << "The buffer received is smaller then expected ("
                << buffer.length << "<" << minBufferLength
                << "), skipping!" << LOGEND;
            videoInterface.releaseImage();
            return;
        }
    } catch(v4l2::V4L2Interface::IOError &e) {
        LOG << "V4L2Interface::IOError: " << e.what() << LOGEND;
        quit();
        return;
    }

    copy_time = rdtsc();

    // if needed, let OpenCV handle the conversion into the right colorspace
    if(conversion != cv::COLOR_COLORCVT_MAX) {
        cv::cvtColor(cv::Mat(inputRows, cols, opencv_convert_type, buffer.start),
                        conversionMatrix,
                        conversion);
        buffer.start = conversionMatrix.data;
    }


    // Split the input image into left and right, line by line. The first
    // line is the right image:
    // (This is tuned to use vector instructions where possible. Don't use
    //  push or similar)

    for(unsigned int outputRow = 0; outputRow < outputRows; outputRow++) {
        unsigned char *start = ((unsigned char *) buffer.start) + 2 * (outputRow * rowLength);
        memcpy(mImageLeft.data + outputRow * rowLength, start             , rowLength);
        memcpy(mImageRight.data + outputRow * rowLength, start + rowLength , rowLength);
    }

    send_time = rdtsc();

    setImageDataFromRaw(&mImageLeft, &mImageRight);

    // release the image buffer
    try {
        videoInterface.releaseImage();
    } catch(v4l2::V4L2Interface::IOError &e) {
        LOG << "V4L2Interface::IOError: " << e.what() << LOGEND;
        quit();
        return;
    }

    if(mProfilingEnabled)
        mProfilingLog << read_time << "," << copy_time << "," << send_time << std::endl;
}

}
