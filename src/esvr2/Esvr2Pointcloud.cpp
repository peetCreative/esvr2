#define USE_POINTCLOUD
#ifdef USE_POINTCLOUD

#include "Esvr2PointCloud.h"

#include "OgreMesh.h"
#include "OgreSubMesh.h"
#include "OgreMeshManager.h"
#include "OgreHardwareVertexBuffer.h"
#include "OgreHardwareBufferManager.h"
#include "OgreVertexIndexData.h"
#include "OgreHardwareVertexBuffer.h"
#include "OgreRoot.h"
#include "OgreRenderOperation.h"
//inspired by: https://forums.ogre3d.org/viewtopic.php?f=5&t=51683

namespace esvr2
{
    PointCloud::PointCloud(
        const std::string& name, const std::string& resourcegroup,
        const int numpoints, float *parray, float *carray)
    {
        /// Create the mesh via the MeshManager
        mMeshPtr = Ogre::v1::MeshManager::getSingleton().createManual(name, resourcegroup);

        /// Create one submesh
        Ogre::v1::SubMesh* sub = mMeshPtr->createSubMesh();

        /// Create vertex data structure for vertices shared between submeshes
        Ogre::v1::VertexData *vtData = new Ogre::v1::VertexData();
        mMeshPtr->sharedVertexData[Ogre::VertexPass::VpNormal] = vtData;
        mMeshPtr->sharedVertexData[Ogre::VertexPass::VpShadow] = vtData;

        /// Create declaration (memory format) of vertex data
        Ogre::v1::VertexDeclaration* decl =
            vtData->vertexDeclaration;

        decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

        vbuf = Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer(
            decl->getVertexSize(0),
            vtData->vertexCount,
            Ogre::v1::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        /// Upload the vertex data to the card
        vbuf->writeData(0, vbuf->getSizeInBytes(), parray, true);

        if(carray != NULL)
        {
            // Create 2nd buffer for colors
            decl->addElement(1, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
            cbuf = Ogre::v1::HardwareBufferManager::getSingleton().createVertexBuffer(
                Ogre::v1::VertexElement::getTypeSize(Ogre::VET_COLOUR),
                vtData->vertexCount,
                Ogre::v1::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

            Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
            Ogre::RGBA *colours = new Ogre::RGBA[numpoints];
            int i = 0, k = 0;
            while( i<numpoints*3 && k<numpoints )
            {
                // Use render system to convert colour value since colour packing varies
                rs->convertColourValue(
                    Ogre::ColourValue(
                        carray[i],carray[i+1],carray[i+2]), &colours[k]);
                i +=3;
                k++;
            }
            // Upload colour data
            cbuf->writeData(0, cbuf->getSizeInBytes(), colours, true);
            delete[] colours;
        }
        /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
        Ogre::v1::VertexBufferBinding* bind =
            vtData->vertexBufferBinding;
        bind->setBinding(0, vbuf);
        if(carray != NULL)
        {
            // Set colour binding so buffer 1 is bound to colour buffer
            bind->setBinding(1, cbuf);
        }
        sub->useSharedVertices = true;
        sub->operationType = Ogre::OperationType::OT_POINT_LIST;
        mMeshPtr->load();
    }
    void PointCloud::updateVertexPositions(int size, float *points)
    {
        float *pPArray = static_cast<float*>(vbuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD));
        for(int i=0; i<size*3; i+=3)
        {
            pPArray[i] = points[i];
            pPArray[i+1] = points[i+1];
            pPArray[i+2] = points[i+2];
        }
        vbuf->unlock();
    }

    void PointCloud::updateVertexColours(int size, float *colours)
    {
        float *pCArray = static_cast<float*>(cbuf->lock(Ogre::v1::HardwareBuffer::HBL_DISCARD));
        for(int i=0; i<size*3; i+=3)
        {
            pCArray[i] = colours[i];
            pCArray[i+1] = colours[i+1];
            pCArray[i+2] = colours[i+2];
        }
        cbuf->unlock();
    }

    Ogre::v1::MeshPtr PointCloud::getMeshPtr()
    {
        return mMeshPtr;
    }

    PointCloud::~PointCloud()
    {
        //TODO: unload mesh
    }
}
#endif
