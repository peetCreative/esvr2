
#ifndef _Esvr2_VideoLoader_H_
#define _Esvr2_VideoLoader_H_

#include "Esvr2.h"

#include <mutex>

namespace esvr2 {
    class GraphicsSystem;

    class VideoLoader
    {
    protected:
        Distortion mDistortion;
//         std::vector<Distortion> mAvailableDistortions;
        bool mStereo;
        int mCur, mLoad;
        int mSeq;
        bool mQuit, mReady;

        size_t mDestinationWidth, mDestinationHeight;
        size_t mDestinationDepth, mDestinationLength;

        Ogre::uint8 *mBuffers[2][2];
        StereoCameraConfig mCameraConfig;

        cv::Mat mUndistortMap1[2], mUndistortMap2[2];
        cv::Mat mUndistortRectifyMap1[2], mUndistortRectifyMap2[2];

        std::mutex mMtx;

        bool configValid();
        bool buffersValid();

        //split image vertical or horizontal
        void setImageDataFromSplit( const cv::Mat *img, Orientation orientation );
        //split and then apply on demand undistortion and rectification
        void setImageDataFromSplitSliced(const cv::Mat *img );
        //apply on demand undistortion and rectification
        void setImageDataFromRaw( cv::Mat *left, cv::Mat *right );
        //simply ImageData
        void setImageData( cv::Mat *left, cv::Mat *right );
    public:
        VideoLoader( Distortion distortion, bool stereo );
        ~VideoLoader();

        //to be overwritten by implementations
        virtual bool initialize( void );
        virtual void deinitialize(void);
        virtual void update( );

        Distortion getDistortion( void ) { return mDistortion; };
        virtual void setDistortion( Distortion distortion )
            { mDistortion = distortion; };

        void quit() {mQuit = true;};
        virtual bool getQuit() {return mQuit;};

        bool isReady();

        bool isStereo() {return mStereo;};

        //should probably be called initialize implementations or by graphics
        bool updateDestinationSize(
            size_t width, size_t height, size_t depth, size_t length );
        void updateMaps();

        StereoCameraConfig getStereoCameraConfig();
        StereoImageData getCurStereoImageData();

        CameraConfig getMonoCameraConfig();
        ImageData getCurMonoImageData();
    };
}

#endif
