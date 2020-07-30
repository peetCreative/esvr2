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

#include <tf2_ros/transform_listener.h>

#include <mutex>

namespace esvr2
{
    VideoROSNode::VideoROSNode(
            int argc, char *argv[],
            RosInputType rosInputType,
            StereoCameraConfig *cameraConfig,
            Distortion distortion, bool stereo,
            std::string rosNamespace,
            std::string rosTopicNameRaw,
            std::string rosTopicNameUndist,
            std::string rosTopicNameUndistRect):
        VideoLoader( distortion, stereo ),
        PoseState(),
        mNh( nullptr ),
        mSubImageLeftRaw( nullptr ),
        mSubImageRightRaw( nullptr ),
        mSubImageLeftUndist( nullptr ),
        mSubImageRightUndist( nullptr ),
        mSubImageLeftUndistRect( nullptr ),
        mSubImageRightUndistRect( nullptr ),
        mApproximateSyncRaw( nullptr ),
        mApproximateSyncUndist( nullptr ),
        mApproximateSyncUndistRect( nullptr ),
        mRosInputType( rosInputType ),
        mRosNamespace( rosNamespace ),
        mRosTopicNameRaw( rosTopicNameRaw ),
        mRosTopicNameUndist( rosTopicNameUndist ),
        mRosTopicNameUndistRect( rosTopicNameUndistRect ),
        mIsCameraInfoInit{ false, false },
        mSubscribePose(true),
        mTfBuffer(nullptr),
        mTfListener(nullptr)
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
        std::string topic;
        switch (mRosInputType)
        {
            case RIT_NONE:
                quit();
                return false;
            case RIT_MONO:
                LOG << "RIT_MONO" << LOGEND;
                topic = mRosNamespace + "image_raw";
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImage = mNh->subscribe(
                    topic, 1,
                    &VideoROSNode::newROSImageMono, this);
                break;
            case RIT_STEREO_SLICED:
                LOG << "RIT_STEREO_SLICED" << LOGEND;
                topic = mRosNamespace + "image";
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImage = mNh->subscribe(
                    topic, 1,
                    &VideoROSNode::newROSImageStereoSliced, this);
                break;
            case RIT_STEREO_SPLIT_RAW:
                LOG << "RIT_STEREO_SPLIT_RAW" << LOGEND;
                topic = mRosNamespace + "left/" + mRosTopicNameRaw;
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImageLeftRaw= new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    topic = mRosNamespace + "right/" + mRosTopicNameRaw;
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImageRightRaw = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                mApproximateSyncRaw.reset(
                    new ApproximateSync(
                        ApproximatePolicy(20),
                        *mSubImageLeftRaw, *mSubImageRightRaw));
                mApproximateSyncRaw->registerCallback(
                    boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                break;
            case RIT_STEREO_SPLIT:
                LOG << "RIT_STEREO_SPLIT" << LOGEND;
                if(mRosTopicNameRaw.compare("") != 0)
                {
                    topic = mRosNamespace + "left/" + mRosTopicNameRaw;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftRaw = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    topic = mRosNamespace + "right/" + mRosTopicNameRaw;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightRaw = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    mApproximateSyncRaw.reset(
                        new ApproximateSync(
                            ApproximatePolicy(20),
                            *mSubImageLeftRaw, *mSubImageRightRaw));
                    mApproximateSyncRaw->registerCallback(
                        boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
                if(mRosTopicNameUndist.compare("") != 0)
                {
                    topic = mRosNamespace + "left/" + mRosTopicNameUndist;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftUndist = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    topic = mRosNamespace + "right/" + mRosTopicNameUndist;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightUndist = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    mApproximateSyncUndist.reset(
                        new ApproximateSync(
                            ApproximatePolicy(20),
                            *mSubImageLeftUndist, *mSubImageRightUndist));
                    mApproximateSyncUndist->registerCallback(
                        boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
                if(mRosTopicNameUndistRect.compare("") != 0)
                {
                    topic = mRosNamespace + "left/" + mRosTopicNameUndistRect;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftUndistRect = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    topic = mRosNamespace + "right/" + mRosTopicNameUndistRect;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightUndistRect = new
                    message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                    mApproximateSyncUndistRect.reset(
                        new ApproximateSync(
                            ApproximatePolicy(20),
                            *mSubImageLeftUndistRect, *mSubImageRightUndistRect));
                    mApproximateSyncUndistRect->registerCallback(
                        boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
        }
        setDistortion( mDistortion );

        if (!mIsCameraInfoInit[LEFT] && !mIsCameraInfoInit[RIGHT])
        {
            topic = mRosNamespace + "left/camera_info";
            LOG << "Subscribe to " << topic << LOGEND;
            mSubCamInfoLeft = mNh->subscribe(
                topic, 1,
                &VideoROSNode::newROSCameraInfoCallback<LEFT>, this);
            topic = mRosNamespace + "right/camera_info";
            LOG << "Subscribe to " << topic << LOGEND;
            mSubCamInfoRight = mNh->subscribe(
                topic, 1,
                &VideoROSNode::newROSCameraInfoCallback<RIGHT>, this);
//             if( mSubCamInfoLeft.getNumPublishers() == 0 ||
//                 mSubCamInfoRight.getNumPublishers() == 0 )
//             {
//                 LOG << "no Publisher for camera_info" << LOGEND;
//                 return false;
//             }
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
            mTfBuffer = new tf2_ros::Buffer();
            mTfListener = new tf2_ros::TransformListener(*mTfBuffer, mNh);
//             topic = "/tf";
//             LOG << "Subscribe to " << topic << LOGEND;
            //TODO: It get's simply not called
//             mSubPose = mNh->subscribe(
//                     topic, 1,
//                     &VideoROSNode::newROSPose, this);
        }
        return true;
    }

    void VideoROSNode::setDistortion( Distortion distortion )
    {
        mDistortion = distortion;
        if (mRosInputType == RIT_STEREO_SPLIT)
        {
            if (mSubImageLeftRaw && mSubImageRightRaw)
            {
                if(mDistortion == DIST_RAW)
                {
                    LOG << "Image set to DIST_RAW" << LOGEND;
                    mSubImageLeftRaw->subscribe();
                    mSubImageRightRaw->subscribe();
                }
                else
                {
                    mSubImageLeftRaw->unsubscribe();
                    mSubImageRightRaw->unsubscribe();
                }
            }
            if ( mSubImageLeftUndist && mSubImageRightUndist )
            {
                if(mDistortion == DIST_UNDISTORT)
                {
                    LOG << "Image set to DIST_UNDISTORT" << LOGEND;
                    mSubImageLeftUndist->subscribe();
                    mSubImageRightUndist->subscribe();
                }
                else
                {
                    mSubImageLeftUndist->unsubscribe();
                    mSubImageRightUndist->unsubscribe();
                }
            }
            if ( mSubImageLeftUndistRect && mSubImageRightUndistRect )
            {
                if(mDistortion == DIST_UNDISTORT_RECTIFY)
                {
                    LOG << "Image set to DIST_UNDISTORT_RECTIFY" << LOGEND;
                    mSubImageLeftUndistRect->subscribe();
                    mSubImageRightUndistRect->subscribe();
                }
                else
                {
                    mSubImageLeftUndistRect->unsubscribe();
                    mSubImageRightUndistRect->unsubscribe();
                }
            }
        }
    }

    void VideoROSNode::newROSPose(
        const geometry_msgs::TransformStamped pose)
    {
        LOG << "getNewPose" << LOGEND;
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
        mSeq = imgRaw->header.seq;
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
        mSeq = imgRaw->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                imgRaw, sensor_msgs::image_encodings::BGR8 );
            cv::Mat image = cv_ptr->image.clone();
            setImageDataFromRaw( &image, nullptr );
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
        mSeq = imgLeft->header.seq;
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
            cv::Mat left = cv_ptr_left->image.clone();
            cv::Mat right = cv_ptr_right->image.clone();
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
            mCameraConfig.cfg[eye].width = camInfo->width;
            mCameraConfig.cfg[eye].height = camInfo->height;
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig.cfg[eye].K[i] = camInfo->K[i];
            for( size_t i = 0; i < 12; i++ )
                mCameraConfig.cfg[eye].P[i] = camInfo->P[i];
            for( size_t i = 0; i < 5; i++ )
                mCameraConfig.cfg[eye].D[i] = camInfo->D[i];
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig.cfg[eye].R[i] = camInfo->R[i];
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
            mReady = true;
        }
    }

    void VideoROSNode::deinitialize(void)
    {
        //TODO: delete all the other things
        ros::shutdown();
    }

    void VideoROSNode::update( )
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped pose;
        bool isTransform = true;
        try{
            if(!mTfBuffer->_frameExists("checkerboard"))
            {
//                 LOG << "no frame named checkerboard" << LOGEND;
                return;
            }
            pose = mTfBuffer->lookupTransform( "checkerboard", "camera", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            isTransform = false;
        }
        if(isTransform)
        {
            Ogre::Vector3 trans(
                pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z);
            Ogre::Quaternion rotation(
                pose.transform.rotation.x, pose.transform.rotation.y,
                pose.transform.rotation.z, pose.transform.rotation.w);
            setPose( trans, rotation );
        }
    }

    bool VideoROSNode::getQuit()
    {
        return !mNh->ok() || mQuit;
    }
}

#endif
