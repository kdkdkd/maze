#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED
#include "Ogre\Ogre.h"
using namespace Ogre;

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

int SolveQuadricEquation(double a, double b, double c, double& x1, double& x2);
std::vector<Vector3> FindBestQuadInsideTriangle(Vector3 a,Vector3 b,Vector3 c);

}

#endif
