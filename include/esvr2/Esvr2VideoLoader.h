
#ifndef _Esvr2_VideoLoader_H_
#define _Esvr2_VideoLoader_H_

#include "Esvr2.h"
#include "Esvr2Component.h"

#include "opencv2/opencv.hpp"
#include <mutex>

namespace esvr2 {
    class GraphicsSystem;

    //! \brief Exchangable superclass to provide Video-Images
    /*! Overwrite the functions from Component and in them the images
     * using the functions:
     * - setImageDataFromSplit
     * - setImageDataFromSplitSliced
     * - setImageDataFromRaw
     * - setImageData
     * \addtogroup Components
     */
    class VideoLoader : virtual public Component
    {
    protected:
        Distortion mDistortion;
//         std::vector<Distortion> mAvailableDistortions;
        bool mStereo;
        int mCur, mLoad;
        int mSeq;

        size_t mDestinationWidth, mDestinationHeight;
        size_t mDestinationDepth, mDestinationLength;

        unsigned char *mBuffers[2][2];
        unsigned char *mBlackBuffer;
        StereoCameraConfig mCameraConfig;

        cv::Mat mUndistortMap1[2], mUndistortMap2[2];
        cv::Mat mUndistortRectifyMap1[2], mUndistortRectifyMap2[2];

        cv::ColorConversionCodes mColorConversion {cv::COLOR_COLORCVT_MAX};

        bool mShow {true};

        std::mutex mMtx;

        bool configValid();
        bool buffersValid();

        //! \brief split image vertical or horizontal
        void setImageDataFromSplit( const cv::Mat *img, Orientation orientation );
        //! \brief split and then apply on demand undistortion and rectification
        void setImageDataFromSplitSliced(const cv::Mat *img );
        //! \brief apply on demand undistortion and rectification
        void setImageDataFromRaw( cv::Mat *left, cv::Mat *right );
        //! \brief simply ImageData
        void setImageData( cv::Mat *left, cv::Mat *right );
    public:
        VideoLoader( Distortion distortion, bool stereo );
        ~VideoLoader();

        Distortion getDistortion( void ) { return mDistortion; };
        virtual void setDistortion( Distortion distortion )
            { mDistortion = distortion; };

        bool isReady();

        bool isStereo() {return mStereo;};

        void setShow(bool show) {mShow = show;};
        bool isShow() {return mShow;};

        //should probably be called initialize implementations or by graphics
        bool updateDestinationSize(
            size_t width, size_t height, size_t depth, size_t length );
        void updateMaps();

        StereoCameraConfig getStereoCameraConfig();
        StereoImageData getCurStereoImageData();

        CameraConfig getMonoCameraConfig();
        ImageData getCurMonoImageData();
    };
    typedef std::shared_ptr<VideoLoader> VideoLoaderPtr;
}

#endif
