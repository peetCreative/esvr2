
#ifndef _Esvr2_VideoLoader_H_
#define _Esvr2_VideoLoader_H_

#include "Esvr2StereoRendering.h"

namespace esvr2 {
    class GraphicsSystem;

    class VideoLoader
    {
    protected:
        Distortion mDistortion;
        bool mStereo;
        int mCur, mLoad;
        int mFrameId;
        bool mQuit, mReady;

        size_t mDestinationWidth, mDestinationHeight;

        Ogre::uint8 *mBuffer[4];
        StereoCameraConfig mCameraConfig;

        cv::Mat mUndistortMap1[2], mUndistortMap2[2];
        cv::Mat mUndistortRectifyMap1[2], mUndistortRectifyMap2[2];

        //split image vertical or horizontal
        void setImageDataFromSplit( const cv::Mat *img, Orientation orientation );
        //split and then apply on demand undistortion and rectification
        void setImageDataFromSplitSliced(const cv::Mat *img );
        //apply on demand undistortion and rectification
        void setImageDataFromRaw( cv::Mat *left, cv::Mat *right );
        //simply ImageData
        void setImageData( cv::Mat *left, cv::Mat *right );
    public:
        VideoLoader( Distortion distortion = DIST_UNDIST_RECT, bool stereo );
        ~VideoLoader();

        //to be overwritten by implementations
        virtual bool initialize( void );
        virtual void deinitialize(void);
        virtual void update( float timeSinceLast );

        Distortion getDistortion( void ) { return mDistortion; };
        virtual void setDistortion( Distortion distortion )
            { mDistortion = distortion; };

        void quit() {mQuit = true;};
        virtual bool getQuit() {return mQuit;};

        bool isReady() {return mReady;};

        bool isStereo() {return mStereo;};

        //should probably be called initialize implementations or by graphics
        bool updateDestinationSize(
            size_t width, size_t height, size_t depth, size_t length );
        bool updateMaps()

        StereoCameraConfig getStereoCameraConfig();
        StereoImageData getCurStereoImageData();

        CameraConfig getMonoCameraConfig();
        ImageData getCurMonoImageData();
    };
}

#endif
