#include "Esvr2VideoLoader.h"

#include "Esvr2.h"
#include <Ogre.h>

namespace esvr2
{
    VideoLoader::VideoLoader( Distortion distortion, bool stereo ):
        Component(),
        mDistortion(distortion),
        mStereo(stereo),
        mCur(0), mLoad(0),
        mSeq(-1),
        mDestinationWidth(0), mDestinationHeight(0),
        mDestinationDepth(0), mDestinationLength(0),
        mBuffers{{nullptr, nullptr},{nullptr, nullptr}}
    {}

    VideoLoader::~VideoLoader()
    {
        for( int i = 0; i < 2; i++ )
            for( int j = 0; j < 2; j++ )
                if( mBuffers[i][j] )
                {
                    //Do not free the pointer if texture's paging strategy is GpuPageOutStrategy::AlwaysKeepSystemRamCopy
                    OGRE_FREE_SIMD( mBuffers[i][j], Ogre::MEMCATEGORY_RESOURCE );
                }
    }


    bool VideoLoader::configValid()
    {
        return mCameraConfig.cfg[LEFT]->valid() &&
            (!mStereo || (mCameraConfig.cfg[RIGHT]->valid() ));
    }

    bool VideoLoader::buffersValid()
    {
        //TODO: also check if valid
        return mBuffers[mCur][LEFT] && mBuffers[mLoad][LEFT] &&
            (!mStereo || (mBuffers[mCur][RIGHT] && mBuffers[mLoad][RIGHT]) );
    }

    bool VideoLoader::isReady()
    {
        //also check the config is valid
        return mReady && configValid() && buffersValid();

    }

    void VideoLoader::setImageDataFromSplit(const cv::Mat *img, Orientation orientation)
    {
        cv::Rect lrect, rrect;
        switch (orientation)
        {
            case ORIENTATION_VERTICAL:
                lrect = cv::Rect( 0, 0,
                                img->cols/2, img->rows);
                rrect = cv::Rect( img->cols/2, 0,
                                img->cols/2, img->rows);
                break;
            case ORIENTATION_HORIZONTAL:
                // left is below
                // right is above
                rrect = cv::Rect( 0, img->rows/2,
                                img->cols, img->rows/2);
                lrect = cv::Rect( 0, 0,
                                img->cols, img->rows/2);
                break;
        }
        cv::Mat imageOrigLeft( *img, lrect );
        cv::Mat imageOrigRight( *img, rrect );
        setImageDataFromRaw(&imageOrigLeft, &imageOrigRight);
    }

    //split and then apply on demand undistortion and rectification
    void VideoLoader::setImageDataFromSplitSliced(const cv::Mat *img)
    {
        if ( !isReady() )
        {
            LOG << "VideoLoader::setImageDataFromSplitSliced: not fully configured" << LOGEND;
            return;
        }
        int outputRows = img->rows/2;
        //imageData is always 4 depth
        int depth = 4 * sizeof(Ogre::uint8);
        if( img->rows % 2 != 0 ) {
            LOG << "Height of input image must be divisible by 2 (but current height is " << img->rows << ")!";
            quit();
            return;
        }
        cv::Mat imgLeft( outputRows,  img->cols, CV_8UC4 );
        cv::Mat imgRight( outputRows,  img->cols, CV_8UC4 );
        // Split the input image into left and right, line by line.
        //TODO: normally First line is right image,
        //but somehow here it is different
        size_t rowLength = depth * img->cols;
        for( int inputRow = 0; inputRow < img->rows; inputRow++ )
        {
            if( inputRow % 2 == 0 )
            {
                size_t outputRow = inputRow/2;
                size_t srcPos = inputRow * img->cols * depth;
                size_t destPos = outputRow * img->cols * depth;
                memcpy( imgRight.data + destPos, img->data + srcPos, rowLength );
            }
            else
            {
                size_t outputRow = (inputRow-1)/2;
                size_t srcPos = inputRow*img->cols*depth;
                size_t destPos = outputRow * img->cols * depth;
                memcpy( imgLeft.data + destPos, img->data + srcPos, rowLength );
            }
        }
        setImageDataFromRaw( &imgLeft, &imgRight);
    }
    //apply on demand undistortion and rectification
    void VideoLoader::setImageDataFromRaw(cv::Mat *left, cv::Mat *right)
    {
        if(!isReady())
        {
            LOG << "VideoLoader::setImageDataFromRaw: not fully configured" << LOGEND;
            return;
        }
        size_t eyeNum = mStereo ? 2 : 1;
        cv::Mat *img[2] = { left, right };
        for (size_t eye = 0; eye < eyeNum; eye++ )
        {
            if ( static_cast<size_t>( img[eye]->cols ) != mCameraConfig.cfg[eye]->width ||
                static_cast<size_t>(  img[eye]->rows ) != mCameraConfig.cfg[eye]->height )
            {
                resize(*(img[eye]), *(img[eye]),
                       cv::Size(mCameraConfig.cfg[eye]->width, mCameraConfig.cfg[eye]->height));
            }
            switch( mDistortion )
            {
                case DIST_RAW:
                    break;
                case DIST_UNDISTORT:
                    cv::remap(
                        *(img[eye]), *(img[eye]),
                              mUndistortMap1[eye], mUndistortMap2[eye],
                              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                    break;
                case DIST_UNDISTORT_RECTIFY:
                    cv::remap(
                        *(img[eye]), *(img[eye]),
                              mUndistortRectifyMap1[eye],
                              mUndistortRectifyMap2[eye],
                              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                    break;
            }
        }
        setImageData(img[LEFT], img[RIGHT]);
    }
    //simply ImageData set to
    void VideoLoader::setImageData(cv::Mat *left, cv::Mat *right)
    {
        if(!isReady())
        {
            LOG << "VideoLoader::setImageData: not fully configured" << LOGEND;
            return;
        }
        size_t eyeNum = mStereo ? 2 : 1;
        cv::Mat *img[2] = { left, right };
        for (size_t eye = 0; eye < eyeNum; eye++ )
        {
            //resize
            if ( static_cast<size_t>( img[eye]->cols ) != mDestinationWidth ||
                static_cast<size_t>(  img[eye]->rows ) != mDestinationHeight )
            {
                resize(*(img[eye]), *(img[eye]),
                       cv::Size(mDestinationWidth, mDestinationHeight));
            }

            if (mColorConversion != cv::COLOR_COLORCVT_MAX )
            {
                //Shouldn't matter whether BGR to BGRA or RGB to RGBA
                cvtColor(*(img[eye]), *(img[eye]), mColorConversion );
            }
            if( img[eye]->type() != CV_8UC4 )
            {
                LOG << "cv::Mat type is wrong" << LOGEND;
                return;
            }
            mMtx.lock();
            if( mDestinationLength != img[eye]->total() * img[eye]->elemSize() )
            {
                LOG << "src Mat has different size than destination requires" << LOGEND;
                return;
            }
            memcpy(mBuffers[mLoad][eye], img[eye]->data, mDestinationLength);
            mMtx.unlock();
        }
        mCur = mLoad;
        // maybe to complicated for toggeling between 0 and 1
        mLoad = (mCur + 1) % 2;
    }

    StereoCameraConfig VideoLoader::getStereoCameraConfig()
    {
        return mCameraConfig;
    }

    StereoImageData VideoLoader::getCurStereoImageData()
    {
        StereoImageData stereoImageData;
        if (!mReady)
            return stereoImageData;
        size_t eyeNum = mStereo ? 2 : 1;
        for (size_t eye = 0; eye < eyeNum; eye++ )
        {
            stereoImageData.img[eye].seq = mSeq;
            stereoImageData.img[eye].width = mDestinationWidth;
            stereoImageData.img[eye].height = mDestinationHeight;
            stereoImageData.img[eye].depth = mDestinationDepth;
            stereoImageData.img[eye].length = mDestinationLength;
            if (mShow)
                stereoImageData.img[eye].data = mBuffers[mCur][eye];
            else
                stereoImageData.img[eye].data = mBlackBuffer;

        }
        return stereoImageData;
    }

    CameraConfig VideoLoader::getMonoCameraConfig()
    {
        return mCameraConfig.leftCameraConfig;
    }
    ImageData VideoLoader::getCurMonoImageData()
    {
        ImageData imageData;
        if(!mReady)
            return imageData;
        imageData.seq = mSeq;
        imageData.width = mDestinationWidth;
        imageData.height = mDestinationHeight;
        imageData.depth = mDestinationDepth;
        imageData.length = mDestinationLength;
        if(mShow)
            imageData.data = mBuffers[mCur][LEFT];
        else
            imageData.data = mBlackBuffer;
        return imageData;
    }

    //COULD BE CALLED AFTER is READY so we maybe should deactivate 
    bool VideoLoader::updateDestinationSize(
        size_t width, size_t height, size_t depth, size_t length)
    {
        if ( length != width * height * depth ) {
            LOG << "Can only set image dimensions, which align nicely" << LOGEND;
            return false;
        }
        mMtx.lock();
        if (length != mDestinationLength)
        {
            for ( int i = 0; i < 2; i++ )
                for( int j = 0; j < 2; j++ )
                {
                    if (mBuffers[i][j])
                    {
                        OGRE_FREE_SIMD(
                            mBuffers[i][j], Ogre::MEMCATEGORY_RESOURCE );
                    }
                    mBuffers[i][j] = reinterpret_cast<Ogre::uint8*>(
                        OGRE_MALLOC_SIMD( length,
                                        Ogre::MEMCATEGORY_RESOURCE ) );
                    memset(mBuffers[i][j], 0, length);
                }
            if (mBlackBuffer)
            {
                OGRE_FREE_SIMD(
                        mBlackBuffer, Ogre::MEMCATEGORY_RESOURCE );
            }
            mBlackBuffer = reinterpret_cast<Ogre::uint8*>(
                    OGRE_MALLOC_SIMD( length,
                                      Ogre::MEMCATEGORY_RESOURCE ) );
            memset(mBlackBuffer, 0, length);
        }
        mDestinationWidth = width;
        mDestinationHeight = height;
        mDestinationDepth = depth;
        mDestinationLength = length;
        mMtx.unlock();
        return true;
    }

    void VideoLoader::updateMaps()
    {
        if(!configValid())
        {
            LOG << "Cannot update maps invalid config" << LOGEND;
            quit();
            return;
        }
        size_t eyeNum = mStereo ? 2 : 1;
        for (size_t eye = 0; eye < eyeNum; eye++ )
        {
            cv::Mat intrinsics = cv::Mat::zeros(3, 3, CV_64FC1);
            for( int y = 0; y < 3; y++ )
                for( int x = 0; x < 3; x++ )
                    intrinsics.at<double>(y,x) = mCameraConfig.cfg[eye]->K[y*3+x];
                cv::Size size( mCameraConfig.cfg[eye]->width, mCameraConfig.cfg[eye]->height );
            cv::initUndistortRectifyMap(
                intrinsics, mCameraConfig.cfg[eye]->D,
                cv::_InputArray(), cv::_InputArray(), size, CV_16SC2,
                                        mUndistortMap1[eye], mUndistortMap2[eye] );
            cv::Mat rectify = cv::Mat::zeros(3, 3, CV_64FC1);
            for( int y = 0; y < 3; y++ )
            {
                for( int x = 0; x < 3; x++ )
                {
                    rectify.at<double>(y,x) = mCameraConfig.cfg[eye]->R[y*3+x];
                }
            }
            cv::Mat project = cv::Mat::zeros(3, 4, CV_64FC1);
            for( int y = 0; y < 3; y++ )
                for( int x = 0; x < 4; x++ )
                    project.at<double>(y,x) = mCameraConfig.cfg[eye]->P[y*4+x];
            cv::initUndistortRectifyMap(
                intrinsics, mCameraConfig.cfg[eye]->D,
                rectify, cv::_InputArray(), size, CV_16SC2,
                mUndistortRectifyMap1[eye], mUndistortRectifyMap2[eye] );
        }
    }
}
