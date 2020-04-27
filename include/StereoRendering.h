
#ifndef _Demo_StereoRendering_H_
#define _Demo_StereoRendering_H_

#define LEFT 0
#define RIGHT 1
#define LOG std::cout
#define LOGEND std::endl

namespace Demo {
    typedef enum {
        WS_TWO_CAMERAS_STEREO,
        WS_INSTANCED_STEREO
    } WorkspaceType;
}

#endif
