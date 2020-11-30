#pragma once


#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "v4l2/v4l2_interface.h"
#include <boost/thread.hpp>
#include <fstream>

namespace esvr2 {

    class LowLatencyVideoLoader : public VideoLoader
    {
        public:
            LowLatencyVideoLoader(
                std::shared_ptr<Esvr2VideoInputConfig> videoInputConfig,
                bool profilingEnabled);
            ~LowLatencyVideoLoader();

            bool initialize( void );
            void deinitialize(void);
            void update( );

        private:
            v4l2::V4L2Interface videoInterface;
            uint64_t read_time, copy_time, send_time;
            bool mProfilingEnabled = false;
            std::ofstream mProfilingLog;
            cv::Mat mImageLeft, mImageRight;
            std::string mPath;
    };

}
