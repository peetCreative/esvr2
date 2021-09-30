
#ifndef _Esvr2_OpenCvVideoLoader_H_
#define _Esvr2_OpenCvVideoLoader_H_

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"


namespace esvr2 {
    class GraphicsSystem;

    //! \brief VideoLoader only using OpenCV functions
    class OpenCvVideoLoader : public VideoLoader
    {
        std::string mPath;
        VideoInputType mVideoInputType;
        cv::VideoCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;

    public:
        OpenCvVideoLoader(
                const VideoInputConfigPtr videoInputConfig);
        ~OpenCvVideoLoader();

        bool initialize() override;
        void deinitialize() override;
        void update(uint64 time);
    };
    typedef std::shared_ptr<OpenCvVideoLoader> OpenCvVideoLoaderPtr;
}

#endif
