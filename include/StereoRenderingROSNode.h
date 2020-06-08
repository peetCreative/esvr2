#ifdef USE_ROS
#ifndef _Demo_StereoRenderingROSNode_H_
#define _Demo_StereoRenderingROSNode_H_

#include "StereoRenderingVideoLoader.h"
#include "StereoRendering.h"

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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

namespace esvr2
{
    class VideoLoader;

    class VideoROSNode : public VideoLoader
    {
    private:
        ros::NodeHandle *mNh;
        ros::Subscriber mSubImage;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        std::shared_ptr<ApproximateSync> mApproximateSync;
        RosInputType mRosInputType;
        //Not the most beautifault solution
        CameraConfig *mCameraConfig;
        std::mutex *mCameraConfigLock;
        bool mIsCameraInfoInit[2];
        bool mQuit;

        void newROSCameraInfoCallback();

    public:
        VideoROSNode(
            StereoGraphicsSystem *graphicsSystem,
            CameraConfig *cameraConfig, std::mutex *cameraConfigLock,
            int argc, char *argv[],
            RosInputType rosInputType );
        ~VideoROSNode();

        void newROSImageStereoSliced( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageMono( const sensor_msgs::Image::ConstPtr& img );
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

        void processIncomingMessage( Demo::Mq::MessageId messageId, const void *data );

        bool getQuit();
    };
}

#endif
#endif
