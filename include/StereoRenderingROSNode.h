#ifdef USE_ROS
#ifndef _Demo_StereoRenderingROSNode_H_
#define _Demo_StereoRenderingROSNode_H_

#include "StereoRenderingVideoLoader.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

namespace Demo
{
    class VideoLoader;

    class VideoROSNode : public VideoLoader
    {
    private:
        ros::NodeHandle mNh;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        //Not the most beautifault solution
        sensor_msgs::CameraInfo cameraInfoCache[2];
        bool mIsCameraInfoInit[2];
        std::shared_ptr<ApproximateSync> mApproximateSync;
        void newROSCameraInfoCallback();
    public:
        VideoROSNode(
            StereoGraphicsSystem *graphicsSystem );
        ~VideoROSNode();

        void newROSImageCallback (
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight );
        void newROSCameraInfoCallbackLeft (
            const sensor_msgs::CameraInfo::ConstPtr& camInfo );
        void newROSCameraInfoCallbackRight (
            const sensor_msgs::CameraInfo::ConstPtr& camInfo );

        void newROSCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& camInfoLeft,
            const sensor_msgs::CameraInfo::ConstPtr& camInfoRight);

        void initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );

        void processIncomingMessage( Mq::MessageId messageId, const void *data );

        bool getQuit();
    };
}

#endif
#endif
