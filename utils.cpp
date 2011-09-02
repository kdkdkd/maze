#include "utils.h"

namespace utils
{

//ROTATE TEXTURE COORDINATES TO GIVEN ANGLE
Vector2 texture_rotate(Vector2 coord,double angle)
{
    Vector2 translate(0.5,0.5);
    Vector2 rot = coord - translate;
    Vector2 res(cos(angle)*rot.x - sin(angle)*rot.y,sin(angle)*rot.x + cos(angle)*rot.y);

    return res + translate;
}

//RETURNS CENTER OF CIRCLE, SAVES POINTS TO RES
Vector3 FindBestQuadInsideTriangle(Vector3 a,Vector3 b,Vector3 c,std::vector<Vector3>& res)
{
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 cb = b - c;

    Vector3 normal = ab.crossProduct(ac);
    Real S = 0.5*(normal).length();
    normal.normalise();
    Real la = cb.length();
    Real lb = ac.length();
    Real lc = ab.length();
    Real p = (la + lb + lc)*0.5;
    Real r = S/p;

    if(!((fabs(normal.x)>0.9 || fabs(normal.y)>0.9 || fabs(normal.z)>0.9) && r>0.9))
    {
        return Vector3::ZERO;
    }
    Vector3 center = a + (lb * ab + lc * ac) / (la + lb + lc);
    Vector3 ar = a - center;
    Vector3 br = b - center;
    Vector3 cr = c - center;

    Real A = ar.y * (br.z - cr.z) + br.y * (cr.z - ar.z) + cr.y * (ar.z - br.z);
    Real B = ar.z * (br.x - cr.x) + br.z  *(cr.x - ar.x) + cr.z * (ar.x - br.x);
    Real C = ar.x * (br.y - cr.y) + br.x * (cr.y - ar.y) + cr.x * (ar.y - br.y);
    Real D = - (ar.x * (br.y * cr.z - cr.y * br.z) + br.x * (cr.y * ar.z - ar.y * cr.z) + cr.x * (ar.y * br.z - br.y * ar.z));





    double z1,z2;
    res.resize(4);
    int index1 = 0;
    int index2 = 2;
    if(fabs(normal.x)>0.9)
    {
        if(normal.x<0)
        {
            index1 = 2;
            index2 = 0;
        }
        utils::SolveQuadricEquation(C*C+A*A,2.0*D*C,D*D - 2.0*r*r*A*A,z1,z2);
        res[index1] = (Vector3((- D - C * z1)/A,0.0,z1) + center);
        res[index1+1] = (Vector3((- D - C * z2)/A,0.0,z2) + center);

        utils::SolveQuadricEquation(B*B+A*A,2.0*B*D,D*D - 2.0*r*r*A*A,z1,z2);
        res[index2] = (Vector3((- D - B * z1)/A,z1,0.0) + center);
        res[index2+1] = (Vector3((- D - B * z2)/A,z2,0.0) + center);
    }
    else if(fabs(normal.y)>0.9)
    {
        if(normal.y<0)
        {
            index1 = 2;
            index2 = 0;
        }

        utils::SolveQuadricEquation(A*A+B*B,2.0*D*A,D*D - 2.0*r*r*B*B,z1,z2);
        res[index1] = (Vector3(z1,(- D - A * z1)/B,0.0) + center);
        res[index1+1] = (Vector3(z2,(- D - A * z2)/B,0.0) + center);

        utils::SolveQuadricEquation(C*C+B*B,2.0*D*C,D*D - 2.0*r*r*B*B,z1,z2);
        res[index2] = (Vector3(0.0,(- D - C * z1)/B,z1) + center);
        res[index2 + 1] = (Vector3(0.0,(- D - C * z2)/B,z2) + center);

    }
    else if(fabs(normal.z)>0.9)
    {
        if(normal.z>0)
        {
            index1 = 2;
            index2 = 0;
        }

        utils::SolveQuadricEquation(C*C+A*A,2.0*D*A,D*D - 2.0*r*r*C*C,z1,z2);
        res[index1] = (Vector3(z1,0.0,(- D - A * z1)/C) + center);
        res[index1 + 1] = (Vector3(z2,0.0,(- D - A * z2)/C) + center);


        utils::SolveQuadricEquation(C*C+B*B,2.0*D*B,D*D - 2.0*r*r*C*C,z1,z2);
        res[index2] = (Vector3(0.0,z1,(- D - B * z1)/C) + center);
        res[index2 + 1] = (Vector3(0.0,z2,(- D - B * z2)/C) + center);

    }
    return center;
}

int SolveQuadricEquation(double a, double b, double c, double& x1, double& x2)
{
    double D = b*b - 4.0*a*c;

    if(D<0)
        return 0;
    else if(D<0.01)
    {
        x1 = - b / (2.0 * a);
        return 1;
    }

    double r1 = - b / (2.0 * a);
    double r2 = sqrt(D) / (2.0 * a);

    x1 = r1 + r2;
    x2 = r1 - r2;

    return 2;
}


void GetMeshInformation(const Ogre::MeshPtr mesh,
                        size_t &vertex_count,
                        Ogre::Vector3* &vertices,
                        size_t &index_count,
                        unsigned long* &indices,
                        const Ogre::Vector3 &position,
                        const Ogre::Quaternion &orient,
                        const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;

    vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh( i );

        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }


    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }


        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index = numTris*3 + index_start;

        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>( offset );
            }

        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[ index_offset++ ] = static_cast<unsigned long>( pShort[k] ) +
                                            static_cast<unsigned long>( offset );
            }

        ibuf->unlock();
        current_offset = next_offset;
    }
}
}
