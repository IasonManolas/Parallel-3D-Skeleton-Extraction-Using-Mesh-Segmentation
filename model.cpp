#include "model.h"
Model::Model(GLchar *path)
{
    std::cout<<"Started Loading Model."<<std::endl;

    this->loadModel(path);

    std::cout<<"Model successfully loaded. "<<std::endl;
    printModelInformation();
}

void Model::Draw()
{
    for(Mesh& mesh:meshes)
        mesh.Draw();
}

void Model::loadModel(std::string path)
{
    Assimp::Importer importer;
    const aiScene* scene=importer.ReadFile(path,aiProcess_Triangulate | aiProcess_FlipUVs|aiProcess_GenNormals);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
            return;
        }
    this->directory = path.substr(0, path.find_last_of('/'));

    this->processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode *node, const aiScene *scene)
{
    // Process all the node's meshes (if any)
    for(GLuint i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        this->meshes.push_back(this->processMesh(mesh, scene));
    }
    //Then do the same for each of its children
    for(GLuint i = 0; i < node->mNumChildren; i++)
    {
        this->processNode(node->mChildren[i], scene);
    }
}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
    // Data to fill
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;

    // Walk through each of the mesh's vertices
    for(GLuint i = 0; i < mesh->mNumVertices; i++)
    {
		Vertex vertex;
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
    // Return a mesh object created from the extracted mesh data
    return Mesh(vertices, indices);
    
}

const void Model::printModelInformation()
{
    int numberOfVerts{0};
    int numberOfFaces{0};
    for(Mesh& mesh:meshes)
    {
        numberOfVerts+=mesh.vertices.size();
        numberOfFaces+=mesh.indices.size()/3;
    }
    std::cout<<"Number of verts:"<<numberOfVerts<<std::endl;
    std::cout<<"Number of faces:"<<numberOfFaces<<std::endl;

}
