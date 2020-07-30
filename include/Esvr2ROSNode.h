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

#include <tf2_ros/transform_listener.h>
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
        ros::Subscriber mSubImage;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftRaw;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightRaw;
        std::shared_ptr<ApproximateSync> mApproximateSyncRaw;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndist;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndist;
        std::shared_ptr<ApproximateSync> mApproximateSyncUndist;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndistRect;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndistRect;
        std::shared_ptr<ApproximateSync> mApproximateSyncUndistRect;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        RosInputType mRosInputType;
        std::string mRosNamespace, mRosTopicNameRaw,
            mRosTopicNameUndist, mRosTopicNameUndistRect;
        std::mutex *mCameraConfigLock;
        bool mIsCameraInfoInit[2];
        bool mSubscribePose;

        tf2_ros::Buffer *mTfBuffer;
        tf2_ros::TransformListener *mTfListener;

        void newROSCameraInfoCallback();

    public:
        VideoROSNode(
            int argc, char *argv[],
            RosInputType rosInputType,
            StereoCameraConfig *cameraConfig,
            Distortion distortion, bool stereo,
            std::string rosNamespace,
            std::string rosTopicNameRaw,
            std::string rosTopicNameUndist,
            std::string rosTopicNameUndistRect );
        ~VideoROSNode();

        void setDistortion( Distortion distortion );

        void newROSPose(const geometry_msgs::TransformStamped pose);
        void newROSImageStereoSliced( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageMono( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageCallback (
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight );
        template <int eye>
        void newROSCameraInfoCallback (
            const sensor_msgs::CameraInfo::ConstPtr& camInfo );

        bool initialize(void);
        void deinitialize(void);
        void update( );

        bool getQuit();
    };
}

#endif
#endif
