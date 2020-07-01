#pragma once

#include "Esvr2VideoLoader.h"
#include "v4l2/v4l2_interface.h"
#include <boost/thread.hpp>
#include <fstream>

namespace esvr2 {

    class LowLatencyVideoLoader : public VideoLoader
    {
        public:
            LowLatencyVideoLoader(
                GraphicsSystem *graphicsSystem,
                VideoInput videoInput, bool profilingEnabled );
            ~LowLatencyVideoLoader();

            virtual void initialize( void );
            virtual void deinitialize(void);
            virtual void update( float timeSinceLast );

        private:
            v4l2::V4L2Interface videoInterface;
            uint64_t read_time, copy_time, send_time;
            VideoInput mVideoInput;
            bool mProfilingEnabled = false;
            std::ofstream mProfilingLog;
            cv::Mat mImageLeft, mImageRight;
    };

}
