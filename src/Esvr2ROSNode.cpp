#ifdef USE_ROS

#include "Esvr2ROSNode.h"

#include "Esvr2StereoRendering.h"

#include "Esvr2PoseState.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2GraphicsSystem.h"
#include "Esvr2GameState.h"

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
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
            int argc, char *argv[],
            RosInputType rosInputType,
            std::string rosNamespace,
            StereoCameraConfig *cameraConfig,
            Distortion distortion, bool stereo):
        VideoLoader( distortion, stereo ),
        PoseState(),
        mNh( nullptr ),
        mSubImageLeft( nullptr ),
        mSubImageRight( nullptr ),
        mApproximateSync( nullptr ),
        mRosInputType( rosInputType ),
        mRosNamespace( rosNamespace ),
        mIsCameraInfoInit{ false, false },
        mSubscribePose(true)
    {
        if (cameraConfig)
        {
            mCameraConfig = *cameraConfig;
            mIsCameraInfoInit[LEFT] = true;
            mIsCameraInfoInit[RIGHT] = true;
        }
        ros::init(argc, argv, "esvr2");
        mNh = new ros::NodeHandle();
    }

    VideoROSNode::~VideoROSNode() {}

    bool VideoROSNode::initialize(void)
    {
        switch (mRosInputType)
        {
            case RIT_NONE:
                quit();
                return false;
            case RIT_MONO:
                mSubImage = mNh->subscribe(
                    mRosNamespace + "image_raw", 1,
                    &VideoROSNode::newROSImageMono, this);
                break;
            case RIT_STEREO_SLICED:
                mSubImage = mNh->subscribe(
                    mRosNamespace + "image", 1,
                    &VideoROSNode::newROSImageStereoSliced, this);
                break;
            case RIT_STEREO_SPLIT_RAW:
                mSubImageLeft = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, mRosNamespace + "left/image_raw", 20);
                mSubImageRight = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, mRosNamespace + "right/image_raw", 20);
                mApproximateSync.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeft, *mSubImageRight));
                mApproximateSync->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                break;
            case RIT_STEREO_SPLIT
                mSubImageLeftRaw = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "left/image_raw", 20);
                mSubImageRightRaw = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "right/image_raw", 20);
                mApproximateSyncRaw.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeftRaw, *mSubImageRightRaw));
                mApproximateSyncRaw->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback<DIST_RAW>, this,_1, _2));
                mSubImageLeftUndist = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "left/image_undist", 20);
                mSubImageRightUndist = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "right/image_undist", 20);
                mApproximateSyncUndist.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeftUndist, *mSubImageRightUndist));
                mApproximateSyncUndist->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback<DIST_UNDSISTORT>, this,_1, _2));
                mSubImageLeftUndistRect = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "left/image_undist_rect", 20);
                mSubImageRightUndistRect = new
                message_filters::Subscriber<sensor_msgs::Image> (
                    *mNh, mRosNamespace + "right/image_undist_rect", 20);
                mApproximateSyncUndistRect.reset(
                    new ApproximateSyncUndistRect(
                        ApproximatePolicy(20),
                        *mSubImageLeftUndistRect, *mSubImageRightUndistRect));
                mApproximateSyncUndistRect->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback<DIST_UNDISTORT_RECTIFY>, this,_1, _2));
                setDistortion( mDistortion );
        }

        if (!mIsCameraInfoInit[LEFT] && !mIsCameraInfoInit[RIGHT])
        {
            mSubCamInfoLeft = mNh->subscribe(
                mRosNamespace + "left/camera_info", 1,
                &VideoROSNode::newROSCameraInfoCallback<LEFT>, this);
            mSubCamInfoRight = mNh->subscribe(
                mRosNamespace + "right/camera_info", 1,
                &VideoROSNode::newROSCameraInfoCallback<RIGHT>, this);
            if( mSubCamInfoLeft.getNumPublishers() == 0 ||
                mSubCamInfoRight.getNumPublishers() == 0 )
            {
                LOG << "no Publisher for camera_info" << LOGEND;
                return false;
            }
            // we have to wait until cameraconfig has been read from messages
            mReady = false;
        }
        else
        {
            updateDestinationSize(
                mCameraConfig.cfg[LEFT].width, mCameraConfig.cfg[LEFT].height, 4u,
                mCameraConfig.cfg[LEFT].width* mCameraConfig.cfg[LEFT].height* 4u );
            updateMaps();
            mReady = true;
        }

        if(mSubscribePose)
        {
            //TODO: It get's simply not called
            mSubPose = mNh->subscribe(
                    "/tf", 1,
                    &VideoROSNode::newROSPose, this);
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
        return true;
    }

    void VideoROSNode::setDistortion( Distortion distortion )
    {
        mDistortion = distortion;
        if (RIT_STEREO_SPLIT)
        {
            switch(mDistortion)
            {
                case DIST_RAW:
                    mSubImageLeftRaw->subscribe();
                    mSubImageRightRaw->subscribe();
                    mSubImageLeftUndist->unsubscribe();
                    mSubImageRightUndist->unsubscribe();
                    mSubImageLeftUndistRect->unsubscribe();
                    mSubImageRightUndistRect->unsubscribe();
                    break;
                case DIST_UNDSISTORT:
                    mSubImageLeftRaw->unsubscribe();
                    mSubImageRightRaw->unsubscribe();
                    mSubImageLeftUndist->subscribe();
                    mSubImageRightUndist->subscribe();
                    mSubImageLeftUndistRect->unsubscribe();
                    mSubImageRightUndistRect->unsubscribe();
                    break;
                case DIST_UNDISTORT_RECTIFY:
                    mSubImageLeftRaw->unsubscribe();
                    mSubImageRightRaw->unsubscribe();
                    mSubImageLeftUndist->unsubscribe();
                    mSubImageRightUndist->unsubscribe();
                    mSubImageLeftUndistRect->subscribe();
                    mSubImageRightUndistRect->subscribe();
                    break;
            }
            mSubImageLeftRaw->subscribe();
            mSubImageRightRaw
        }
        //TODO: subscribe to the new correct topic
    }

    void VideoROSNode::newROSPose(
        const geometry_msgs::TransformStamped pose)
    {
        Ogre::Vector3 trans(
            pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z);
        Ogre::Quaternion rotation(
            pose.transform.rotation.x, pose.transform.rotation.y,
            pose.transform.rotation.z, pose.transform.rotation.w);
        setPose( trans, rotation );
    }

    void VideoROSNode::newROSImageStereoSliced(
        const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgLeft->header.seq
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
        setImageDataFromSplitSliced(&(cv_ptr->image));

    }

    void VideoROSNode::newROSImageMono(
        const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgLeft->header.seq
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                imgRaw, sensor_msgs::image_encodings::BGR8 );
            setImageDataFromRaw(
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
        mSeq = imgLeft->header.seq
        cv_bridge::CvImageConstPtr cv_ptr_left;
        cv_bridge::CvImageConstPtr cv_ptr_right;
        try
        {
            //TRY BGRA8
            cv_ptr_left = cv_bridge::toCvShare(
                imgLeft, sensor_msgs::image_encodings::BGR8 );
            cv_ptr_right = cv_bridge::toCvShare(
                imgRight, sensor_msgs::image_encodings::BGR8 );
            //TODO:copy to new mat probably inefficient
            cv::Mat left = cv_ptr_left->clone();
            cv::Mat right = cv_ptr_right->clone();
            if ( mRosInputType == RIT_STEREO_SPLIT_RAW )
            {
                setImageDataFromRaw(
                    &(left), &(right));
            }
            else if ( mRosInputType == RIT_STEREO_SPLIT )
            {
                setImageData(&(left), &(right));
            }
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
            updateDestinationSize(
                mCameraConfig.cfg[LEFT].width, mCameraConfig.cfg[LEFT].height, 4u,
                mCameraConfig.cfg[LEFT].width* mCameraConfig.cfg[LEFT].height* 4u );
            updateMaps();
        }
    }

    void VideoROSNode::deinitialize(void)
    {
        ros::shutdown();
    }

    void VideoROSNode::update( )
    {
        ros::spinOnce();
    }

    bool VideoROSNode::getQuit()
    {
        return !mNh->ok() || getQuit();
    }
}

#endif
