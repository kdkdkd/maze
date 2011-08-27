#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED
#include "Ogre\Ogre.h"
namespace utils
{

void GetMeshInformation(const Ogre::MeshPtr mesh,
                        size_t &vertex_count,
                        Ogre::Vector3* &vertices,
                        size_t &index_count,
                        unsigned long* &indices,
                        const Ogre::Vector3 &position,
                        const Ogre::Quaternion &orient,
                        const Ogre::Vector3 &scale);

}

#endif
