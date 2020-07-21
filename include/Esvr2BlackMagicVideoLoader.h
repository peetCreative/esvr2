
#ifndef _Esvr2_BlackMagicVideoLoader_H_
#define _Esvr2_BlackMagicVideoLoader_H_
#ifdef USE_BLACKMAGICCAMERA
#include "Esvr2StereoRendering.h"

#include "Esvr2VideoLoader.h"

#include "opencv2/opencv.hpp"

#include "BlackMagicCapture.h"

namespace esvr2 {
    class GraphicsSystem;

    class BlackMagicVideoLoader : public VideoLoader
    {
        VideoInput mVideoInput;
        CBlackMagicCapture mCapture;
        int mCaptureFrameWidth;
        int mCaptureFrameHeight;

    public:
        BlackMagicVideoLoader(
            VideoInput vInput,
            StereoCameraConfig cameraConfig,
            bool stereo);
        ~BlackMagicVideoLoader();

        void initialize(void);
        void deinitialize(void);
        void update( float timeSinceLast );
    };
}

#endif
#endif
