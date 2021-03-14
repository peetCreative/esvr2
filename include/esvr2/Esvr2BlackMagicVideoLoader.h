
#ifndef _Esvr2_BlackMagicVideoLoader_H_
#define _Esvr2_BlackMagicVideoLoader_H_
#ifdef USE_BLACKMAGICCAMERA
#include "Esvr2.h"
#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"

#include "BlackMagicCapture.h"

namespace esvr2 {
    class GraphicsSystem;

    class BlackMagicVideoLoader : public VideoLoader
    {
        VideoInputConfigPtr mVideoInputConfig;
        CBlackMagicCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;

    public:
        BlackMagicVideoLoader(
                VideoInputConfigPtr videoInputConfig);
        ~BlackMagicVideoLoader();

        bool initialize(void);
        void deinitialize(void);
        void update( uint64 time );
    };
    typedef std::shared_ptr<BlackMagicVideoLoader> BlackMagicVideoLoaderPtr;
}

#endif
#endif
