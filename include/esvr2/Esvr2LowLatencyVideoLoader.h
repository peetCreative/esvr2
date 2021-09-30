#ifdef USE_LOWLATENCYVIDEOLOADER
#pragma once


#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "v4l2/v4l2_interface.h"
#include <boost/thread.hpp>
#include <fstream>

namespace esvr2 {

    //! \brief a VideoLoader for Capturecards featuring Video4Linux
    /*!
     * \inherit VideoLoader
     */
class LowLatencyVideoLoader : public VideoLoader
    {
        public:
            LowLatencyVideoLoader(
                VideoInputConfigPtr videoInputConfig,
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
    typedef std::shared_ptr<LowLatencyVideoLoader> LowLatencyVideoLoaderPtr;

}
#endif
