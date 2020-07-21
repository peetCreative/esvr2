#pragma once

#include "Esvr2VideoLoader.h"
#include "Esvr2StereoRendering.h"
#include "v4l2/v4l2_interface.h"
#include <boost/thread.hpp>
#include <fstream>

namespace esvr2 {

    class LowLatencyVideoLoader : public VideoLoader
    {
        public:
            LowLatencyVideoLoader(
                VideoInput videoInput, bool profilingEnabled,
                StereoCameraConfig cameraConfig,
                Distortion distortion);
            ~LowLatencyVideoLoader();

            bool initialize( void );
            void deinitialize(void);
            void update( float timeSinceLast );

        private:
            v4l2::V4L2Interface videoInterface;
            uint64_t read_time, copy_time, send_time;
            VideoInput mVideoInput;
            bool mProfilingEnabled = false;
            std::ofstream mProfilingLog;
            cv::Mat mImageLeft, mImageRight;
    };

}
