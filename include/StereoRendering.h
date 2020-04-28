
#ifndef _Demo_StereoRendering_H_
#define _Demo_StereoRendering_H_

#include "OgreMatrix4.h"

#define LEFT 0
#define RIGHT 1
#define LOG std::cout
#define LOGEND std::endl

namespace Demo {
    typedef enum {
        WS_TWO_CAMERAS_STEREO,
        WS_INSTANCED_STEREO
    } WorkspaceType;
    typedef struct {
        Ogre::Matrix4 eyeToHead[2];
        Ogre::Matrix4 projectionMatrix[2];
        Ogre::Vector4 tan[2];
    } HmdConfig;
    typedef struct {
        int width[2] = {0,0};
        int height[2] = {0, 0};
        float f_x[2] = {0, 0};
        float f_y[2] = {0, 0};
        float c_x[2] = {0, 0};
        float c_y[2] = {0, 0};
    } CameraConfig;
}

#endif
