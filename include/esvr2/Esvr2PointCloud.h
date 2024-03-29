/** \file Esvr2PointCloud.h File containing a Class for visualisation of pointclouds.
 * The class can be intialized with a array of points organized like: [0]=P1x [1]=P1y [2]=P1z
 * [3]=P2x [4]=P2y [5]=P2z ...
 * \deprecated This feature is atm not used or tested
 */
#ifndef _Esvr2_POINTCLOUD_H_
#define _Esvr2_POINTCLOUD_H_
#ifdef USE_POINTCLOUD

#include "Esvr2.h"
#include "OgreHardwareVertexBuffer.h"

namespace esvr2
{
    class PointCloud
    {
    public:
        PointCloud(
            const std::string& name,
            const std::string& resourcegroup,
            const int numpoints, Ogre::Real *parray, Ogre::Real *carray);
        ~PointCloud();

        /// Update a created pointcloud with size points.
        void updateVertexPositions(int size, Ogre::Real *points);

        /// Update vertex colours
        void updateVertexColours(int size, Ogre::Real *colours);

        Ogre::v1::MeshPtr getMeshPtr( void );

    private:
        Ogre::v1::MeshPtr mMeshPtr;
        int mSize;
        Ogre::v1::HardwareVertexBufferSharedPtr vbuf;
        Ogre::v1::HardwareVertexBufferSharedPtr cbuf;
    };
}

#endif
#endif
