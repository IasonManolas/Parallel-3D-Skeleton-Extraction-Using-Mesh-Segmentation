#ifndef MESHLOADER_H
#define MESHLOADER_H
#include <vector>
#include <tuple>
#include <random>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/vec3.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Point_3.h>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point=Kernel::Point_3;
using CGALSurfaceMesh=CGAL::Surface_mesh<Point>;

namespace meshLoader{
inline void processMeshAssimp(aiMesh *mesh,std::vector<uint>& indices,std::vector<MyVertex>& vertices)
{

    // Walk through each of the mesh's vertices
    for(GLuint i = 0; i < mesh->mNumVertices; i++)
    {
        MyVertex vertex;
        glm::vec3 vector; // We declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
        // Positions
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex.Position = vector;
        // Normals
        vector.x = mesh->mNormals[i].x;
        vector.y = mesh->mNormals[i].y;
        vector.z = mesh->mNormals[i].z;
        vertex.Normal = vector;
        vertices.push_back(vertex);
    }
    // Now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(GLuint i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        // Retrieve all indices of the face and store them in the indices vector
        for(GLuint j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }
}


inline void processNodeAssimp(aiNode *node, const aiScene *scene,std::vector<uint>& indices,std::vector<MyVertex>& vertices)
{
        // Process all the node's meshes (if any)
    for(GLuint i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMeshAssimp(mesh,indices,vertices);
    }
    //Then do the same for each of its children
    for(GLuint i = 0; i < node->mNumChildren; i++)
    {
        processNodeAssimp(node->mChildren[i], scene, indices, vertices);
    }

}

inline void loadMeshAssimp(std::string path,std::vector<uint>& indices,std::vector<MyVertex>& vertices )
{
      Assimp::Importer importer;
    const aiScene* scene=importer.ReadFile(path,aiProcess_Triangulate | aiProcess_FlipUVs|aiProcess_GenSmoothNormals);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
//            std::cout << "ERROR::ASSIMP::" << importer.GetImporterInfo(GetImporterCount() << std::endl;
            return;
        }
//    this->directory = path.substr(0, path.find_last_of('/'));

    processNodeAssimp(scene->mRootNode, scene,indices,vertices);

}


inline std::tuple<std::vector<uint>,std::vector<MyVertex>> load(std::string filename)
{
    std::cout <<"Loading "<<filename<<" using Assimp library."<<std::endl;
   std::vector<uint> 		indices;
   std::vector<MyVertex> 	vertices;
   loadMeshAssimp(filename,indices,vertices);

   if(indices.size()!=0 && vertices.size()!=0)
       std::cout<<"Loading was successfull."<<std::endl;

   return std::make_tuple(indices,vertices);

}

}
namespace meshMeasuring {

 inline float findMaxDimension(std::vector<MyVertex> vertices)
    {
        float maxDim;
        std::cout<<"Normalizing mesh.."<<std::endl;
        std::vector<MyVertex>::iterator first,last;
        first=vertices.begin();
        last=vertices.end();
        if(first==last) maxDim=1;

        std::vector<MyVertex>::iterator xmin,xmax,ymin,ymax,zmin,zmax;

        xmin=first;
        xmax=first;
        ymin=first;
        ymax=first;
        zmin=first;
        zmax=first;

        while(++first!=last)
        {
        if((*first).Position.x<(*xmin).Position.x)
            xmin=first;
        else if((*first).Position.x>(*xmax).Position.x)
            xmax=first;

        if((*first).Position.y<(*ymin).Position.y)
            ymin=first;
        else if((*first).Position.y>(*ymax).Position.y)
            ymax=first;

        if((*first).Position.z<(*zmin).Position.z)
            zmin=first;
        else if((*first).Position.z>(*zmax).Position.z)
            zmax=first;
        }
        maxDim=std::max(std::max((*xmax).Position.x-(*xmin).Position.x,(*ymax).Position.y-(*ymin).Position.y),(*zmax).Position.z-(*zmin).Position.z);
        std::cout<<"finished mesh normalization."<<std::endl;
        return maxDim;
    }
inline glm::vec3 findCenterOfMass(std::vector<MyVertex> vertices)
    {
        glm::vec3 centerOfMass;
        std::cout<<"Computing center of mass.."<<std::endl;
        glm::vec3 sum{0,0,0};
        for(const MyVertex& vertex:vertices)
            sum+=vertex.Position;
        uint n=vertices.size();
        centerOfMass={sum.x/n,sum.y/n,sum.z/n};
        std::cout<<"Finished computing center of mass"<<std::endl;
        return centerOfMass;
    }

inline float findAverageEdgeLength(const CGALSurfaceMesh& M)
{
        std::cout<<"Computing average edge length.."<<std::endl;
    CGALSurfaceMesh::vertex_iterator vi=M.vertices_begin();

    int numOfSamples=100;
    double sum=0;
    for(int i=0;i<numOfSamples;i++){
        int vertexIndex=rand()%M.number_of_vertices();

        CGALSurfaceMesh::Halfedge_index hIndex=M.halfedge(vertexIndex);
        CGALSurfaceMesh::Halfedge_index prevHIndex=M.prev(hIndex);
        const Point& a=M.point(M.source(prevHIndex));
        const Point& b=M.point(M.source(hIndex));
        sum+=CGAL::sqrt(CGAL::squared_distance(a,b));
    }
    double meanEdgeLength=sum/numOfSamples;
    return meanEdgeLength;
}
}
#endif // MESHLOADER_H
