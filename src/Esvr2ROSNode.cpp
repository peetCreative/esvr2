#ifdef USE_ROS

#include "Esvr2ROSNode.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2GameState.h"

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <mutex>

namespace esvr2
{
    VideoROSNode::VideoROSNode(
            GraphicsSystem *graphicsSystem,
            StereoCameraConfig *cameraConfig, std::mutex *cameraConfigLock,
            int argc, char *argv[],
            RosInputType rosInputType ):
        VideoLoader( graphicsSystem ),
        mNh( nullptr ),
        mSubImageLeft( nullptr ),
        mSubImageRight( nullptr ),
        mApproximateSync( nullptr ),
        mRosInputType( rosInputType ),
        mCameraConfig( cameraConfig ),
        mCameraConfigLock( cameraConfigLock ),
        mIsCameraInfoInit{ false, false }
    {
        ros::init(argc, argv, "esvr2");
        mNh = new ros::NodeHandle();
    }

    VideoROSNode::~VideoROSNode() {}

    void VideoROSNode::initialize(void)
    {
        switch (mRosInputType)
        {
            case ROS_NONE:
                quit();
                break;
            case ROS_MONO:
                mSubImage = mNh->subscribe(
                    "/stereo/camera_driver/image_raw", 1,
                    &VideoROSNode::newROSImageMono, this);
                break;
            case ROS_STEREO_SLICED:
                mSubImage = mNh->subscribe(
                    "/stereo/camera_driver/image", 1,
                    &VideoROSNode::newROSImageStereoSliced, this);
                break;
            case ROS_STEREO_SPLIT:
                mSubImageLeft = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, "/stereo/left/image", 20);
                mSubImageRight = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, "/stereo/right/image", 20);
                mApproximateSync.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeft, *mSubImageRight));
                mApproximateSync->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                if (mCameraConfig && mCameraConfigLock)
                {
                    mSubCamInfoLeft = mNh->subscribe(
                        "/stereo/left/camera_info", 1,
                        &VideoROSNode::newROSCameraInfoCallback<LEFT>, this);
                    mSubCamInfoRight = mNh->subscribe(
                        "/stereo/right/camera_info", 1,
                        &VideoROSNode::newROSCameraInfoCallback<RIGHT>, this);
                }
                break;
        }
        if( mCameraConfigLock && mCameraConfig )
        {
            while(!mCameraConfigLock->try_lock())
                ros::spinOnce();
            mCameraConfigLock->unlock();
        }
        else
        {
            mIsCameraInfoInit[LEFT] = true;
            mIsCameraInfoInit[RIGHT] = true;
        }
    }

    void VideoROSNode::newROSImageStereoSliced(
        const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                imgRaw, sensor_msgs::image_encodings::BGR8 );
        }
        catch (cv_bridge::Exception& e)
        {
            LOG << "cv_bridge exception:" << e.what() << LOGEND;
            return;
        }

        size_t inputRows = (size_t) cv_ptr->image.rows;
        size_t cols = (size_t) cv_ptr->image.cols;
        size_t outputRows = inputRows/2;

        if( inputRows % 2 != 0 ) {
            LOG << "Height of input image must be divisible by 2 (but current height is " << inputRows  << ")!";
            mQuit = true;
            return;
        }

        cv::Mat leftImage( outputRows, cols, CV_8UC3 );
        cv::Mat rightImage( outputRows, cols, CV_8UC3 );

        // Split the input image into left and right, line by line.
        //TODO: normally First line is right image,
        //but somehow here it is different
        for( size_t inputRow = 0; inputRow < inputRows; inputRow++ )
        {
            if( inputRow % 2 == 0 )
            {
                int outputRow = inputRow/2;
                unsigned int srcPos = inputRow * cols * sizeof(unsigned char) *3;
                unsigned int destPos = outputRow*cols*sizeof(unsigned char)*3;
                memcpy( leftImage.data + destPos, cv_ptr->image.data + srcPos, sizeof(unsigned char)*3*cols );
            }
            else
            {
                int outputRow = (inputRow-1)/2;
                unsigned int srcPos = inputRow*cols*sizeof(unsigned char)*3;
                unsigned int destPos = outputRow*cols*sizeof(unsigned char)*3;
                memcpy( rightImage.data + destPos, cv_ptr->image.data + srcPos, sizeof(unsigned char)*3*cols );
            }
        }
        mGraphicsSystem->setImgPtr( &leftImage, &rightImage );
    }

    void VideoROSNode::newROSImageMono(
        const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                imgRaw, sensor_msgs::image_encodings::BGR8 );
            mGraphicsSystem->setImgPtr(
                &(cv_ptr->image), nullptr);
        }
        catch (cv_bridge::Exception& e)
        {
            quit();
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
        catch( Ogre::Exception &e )
        {
            quit();
            std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
            //TODO: let's unregister this as well
            throw e;
        }
        catch( ... )
        {
            quit();
//             destroySystems( graphicsGameState, graphicsSystem );
        }

    }

    void VideoROSNode::newROSImageCallback(
        const sensor_msgs::Image::ConstPtr& imgLeft,
        const sensor_msgs::Image::ConstPtr& imgRight )
    {
        if (!mIsCameraInfoInit[LEFT] || !mIsCameraInfoInit[RIGHT])
            return;
        cv_bridge::CvImageConstPtr cv_ptr_left;
        cv_bridge::CvImageConstPtr cv_ptr_right;
        try
        {
            //TRY BGRA8
            cv_ptr_left = cv_bridge::toCvShare(
                imgLeft, sensor_msgs::image_encodings::BGR8 );
            cv_ptr_right = cv_bridge::toCvShare(
                imgRight, sensor_msgs::image_encodings::BGR8 );
            mGraphicsSystem->setImgPtr(
                &(cv_ptr_left->image), &(cv_ptr_right->image));
        }
        catch (cv_bridge::Exception& e)
        {
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
        catch( Ogre::Exception &e )
        {
            std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
            //TODO: let's unregister this as well
            throw e;
        }
        catch( ... )
        {
//             destroySystems( graphicsGameState, graphicsSystem );
        }
    }

    template<int eye>
    void VideoROSNode::newROSCameraInfoCallback(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        LOG << "camera_info " << eye << LOGEND;
        if ( !mIsCameraInfoInit[eye] )
        {
            mCameraConfig->cfg[eye].width = camInfo->width;
            mCameraConfig->cfg[eye].height = camInfo->height;
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig->cfg[eye].K[i] = camInfo->K[i];
            for( size_t i = 0; i < 12; i++ )
                mCameraConfig->cfg[eye].P[i] = camInfo->P[i];
            for( size_t i = 0; i < 5; i++ )
                mCameraConfig->cfg[eye].D[i] = camInfo->D[i];
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig->cfg[eye].R[i] = camInfo->R[i];
            mIsCameraInfoInit[eye] = true;
        }
        if (eye == LEFT)
        {
            mSubCamInfoLeft.shutdown();
        }
        else if (eye == RIGHT)
        {
            mSubCamInfoRight.shutdown();
        }
        if ( mIsCameraInfoInit[LEFT] && mIsCameraInfoInit[RIGHT] )
        {
            mCameraConfigLock->unlock();
        }
    }

    void VideoROSNode::deinitialize(void)
    {
        ros::shutdown();
    }

    void VideoROSNode::update( float timeSinceLast )
    {
        ros::spinOnce();
    }

    void VideoROSNode::processIncomingMessage(
        Demo::Mq::MessageId messageId, const void *data )
    {}

    bool VideoROSNode::getQuit()
    {
        return !mNh->ok() || getQuit();
    }
}

#endif
