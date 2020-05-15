#ifdef USE_ROS

#include "StereoRenderingROSNode.h"
#include "StereoRenderingGraphicsSystem.h"
#include "StereoRendering.h"

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


namespace Demo
{
    VideoROSNode::VideoROSNode(
            StereoGraphicsSystem *graphicsSystem,
            int argc, char *argv[],
            RosInputType rosInputType ):
        VideoLoader(graphicsSystem),
        mNh( nullptr ),
        mSubImageLeft( nullptr ),
        mSubImageRight( nullptr ),
        mApproximateSync( nullptr ),
        mRosInputType( rosInputType ),
        mIsCameraInfoInit{ false, false },
        mQuit( false )
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
                mQuit = true;
                break;
            case ROS_MONO:
                mSubImage = mNh->subscribe(
                    "/image_raw", 1,
                    &VideoROSNode::newROSImageMono, this);
                break;
            case ROS_STEREO_SLICED:
                mSubImage = mNh->subscribe(
                    "/stereo/image_raw", 1,
                    &VideoROSNode::newROSImageStereoSliced, this);
                break;
            case ROS_STEREO_SPLIT:
                mSubImageLeft = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, "/stereo/left/image_undist_rect", 20);
                mSubImageRight = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, "/stereo/right/image_undist_rect", 20);
                mApproximateSync.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeft, *mSubImageRight));
                mApproximateSync->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                mSubCamInfoLeft = mNh->subscribe(
                    "/stereo/left/camera_info", 1,
                    &VideoROSNode::newROSCameraInfoCallbackLeft, this);
                mSubCamInfoRight = mNh->subscribe(
                    "/stereo/right/camera_info", 1,
                    &VideoROSNode::newROSCameraInfoCallbackRight, this);
                break;
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

        // Split the input image into left and right, line by line. First line is right image:
        for( size_t inputRow = 0; inputRow < inputRows; inputRow++ )
        {
            if( inputRow % 2 == 0 )
            {
                int outputRow = inputRow/2;
                unsigned int srcPos = inputRow * cols * sizeof(unsigned char) *3;
                unsigned int destPos = outputRow*cols*sizeof(unsigned char)*3;
                memcpy( rightImage.data + destPos, cv_ptr->image.data + srcPos, sizeof(unsigned char)*3*cols );
            }
            else
            {
                int outputRow = (inputRow-1)/2;
                unsigned int srcPos = inputRow*cols*sizeof(unsigned char)*3;
                unsigned int destPos = outputRow*cols*sizeof(unsigned char)*3;
                memcpy( leftImage.data + destPos, cv_ptr->image.data + srcPos, sizeof(unsigned char)*3*cols );
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
            mQuit = true;
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
        catch( Ogre::Exception &e )
        {
            mQuit = true;
            std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
            //TODO: let's unregister this as well
            throw e;
        }
        catch( ... )
        {
            mQuit = true;
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

    void VideoROSNode::newROSCameraInfoCallbackLeft(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        std::cout << "camera_info_left" << std::endl;
        if (!mIsCameraInfoInit[LEFT])
        {
            std::cout << "camera_info_left" << std::endl;
            cameraInfoCache[LEFT] = *camInfo;
            mIsCameraInfoInit[LEFT] = true;
            mSubCamInfoLeft.shutdown();
        }
        if ( mIsCameraInfoInit[RIGHT] )
        {
            newROSCameraInfoCallback();
        }
    }
    void VideoROSNode::newROSCameraInfoCallbackRight(
        const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        std::cout << "camera_info_right" << std::endl;
        if (!mIsCameraInfoInit[RIGHT])
        {
            std::cout << "camera_info_right" << std::endl;
            cameraInfoCache[RIGHT] = *camInfo;
            mIsCameraInfoInit[RIGHT] = true;
            mSubCamInfoRight.shutdown();
        }
        if ( mIsCameraInfoInit[RIGHT] )
        {
            newROSCameraInfoCallback();
        }
    }

    void VideoROSNode::newROSCameraInfoCallback()
    {
        if (!mIsCameraInfoInit[LEFT] || !mIsCameraInfoInit[RIGHT])
            return;
        CameraConfig cc = {
            { cameraInfoCache[LEFT].width, cameraInfoCache[RIGHT].width },
            { cameraInfoCache[LEFT].height, cameraInfoCache[RIGHT].height },
            { cameraInfoCache[LEFT].K[0], cameraInfoCache[RIGHT].K[0] },
            { cameraInfoCache[LEFT].K[4], cameraInfoCache[RIGHT].K[4] },
            { cameraInfoCache[LEFT].K[2], cameraInfoCache[RIGHT].K[2] },
            { cameraInfoCache[LEFT].K[5], cameraInfoCache[RIGHT].K[5] }
        };
        mGraphicsSystem->calcAlign( cc );
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
        Mq::MessageId messageId, const void *data )
    {}

    bool VideoROSNode::getQuit()
    {
        return !mNh->ok() || mQuit;
    }
}

#endif
