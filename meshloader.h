#ifndef MESHLOADER_H
#define MESHLOADER_H
#include <vector>
#include <tuple>

//struct MyVertex {
//    glm::vec3 Position;
//    glm::vec3 Normal;

//};
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
    const aiScene* scene=importer.ReadFile(path,aiProcess_Triangulate | aiProcess_FlipUVs|aiProcess_GenNormals);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
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
#endif // MESHLOADER_H
