#ifdef USE_ROS
#ifndef _Esvr2_ROSNode_H_
#define _Esvr2_ROSNode_H_

#include "Esvr2VideoLoader.h"
#include "Esvr2PoseState.h"
#include "Esvr2StereoRendering.h"

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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

namespace esvr2
{
    class VideoLoader;

    class VideoROSNode : public VideoLoader, public PoseState
    {
    private:
        ros::NodeHandle *mNh;
        ros::Subscriber mSubPose;
        ros::Subscriber mSubImage;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeft;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRight;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        std::shared_ptr<ApproximateSync> mApproximateSync;
        RosInputType mRosInputType;
        std::string mRosNamespace;
        //Not the most beautifault solution
        StereoCameraConfig *mCameraConfig;
        std::mutex *mCameraConfigLock;
        bool mIsCameraInfoInit[2];
        bool mSubscribePose;
        bool mQuit;

        void newROSCameraInfoCallback();

    public:
        VideoROSNode(
            GraphicsSystem *graphicsSystem,
            StereoCameraConfig *cameraConfig, std::mutex *cameraConfigLock,
            int argc, char *argv[],
            RosInputType rosInputType,
            std::string rosNamespace);
        ~VideoROSNode();

        void newROSPose(const geometry_msgs::TransformStamped pose);
        void newROSImageStereoSliced( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageMono( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageCallback (
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight );
        template <int eye>
        void newROSCameraInfoCallback (
            const sensor_msgs::CameraInfo::ConstPtr& camInfo );

        void initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );

        void processIncomingMessage( Demo::Mq::MessageId messageId, const void *data );

        bool getQuit();
    };
}

#endif
#endif
