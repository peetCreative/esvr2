/** Class for visualisation of pointclouds.
 * The class can be intialized with a array of points organized like: [0]=P1x [1]=P1y [2]=P1z
 * [3]=P2x [4]=P2y [5]=P2z ...
 */
#ifndef _POINTCLOUD_H_
#define _POINTCLOUD_H_
#define USE_POINTCLOUD
#ifdef USE_POINTCLOUD

#include "OgreHardwareVertexBuffer.h"

namespace esvr2
{
    class PointCloud
    {
    public:
        PointCloud(
            const std::string& name,
            const std::string& resourcegroup,
            const int numpoints, float *parray, float *carray);
        ~PointCloud();

        /// Update a created pointcloud with size points.
        void updateVertexPositions(int size, float *points);

        /// Update vertex colours
        void updateVertexColours(int size, float *colours);

    private:
        int mSize;
        Ogre::v1::HardwareVertexBufferSharedPtr vbuf;
        Ogre::v1::HardwareVertexBufferSharedPtr cbuf;
    };
}

#endif
#endif
