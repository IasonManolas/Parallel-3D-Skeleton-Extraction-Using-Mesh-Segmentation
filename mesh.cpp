#include "mesh.h"

Mesh::Mesh(GLchar *path)
{
    std::cout<<"Started Loading Model."<<std::endl;

    this->loadMeshAssimp(path);


    std::cout<<"Model successfully loaded. "<<std::endl;

    findCenterOfMass();
    findMaxDim();
    updateModelMatrix();
    std::cout<<"My maxDim is:"<<maxDim<<std::endl;
    printMeshInformation();

    setupMesh();

}

void Mesh::loadMeshAssimp(std::__cxx11::string path)
{
      Assimp::Importer importer;
    const aiScene* scene=importer.ReadFile(path,aiProcess_Triangulate | aiProcess_FlipUVs|aiProcess_GenNormals);

    if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cout << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
            return;
        }
    this->directory = path.substr(0, path.find_last_of('/'));

    this->processNodeAssimp(scene->mRootNode, scene);

}

void Mesh::processNodeAssimp(aiNode *node, const aiScene *scene)
{
        // Process all the node's meshes (if any)
    for(GLuint i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        this->processMeshAssimp(mesh);
    }
    //Then do the same for each of its children
    for(GLuint i = 0; i < node->mNumChildren; i++)
    {
        this->processNodeAssimp(node->mChildren[i], scene);
    }

}

void Mesh::processMeshAssimp(aiMesh *mesh)
{

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
        this->vertices.push_back(vertex);
    }
    // Now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(GLuint i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        // Retrieve all indices of the face and store them in the indices vector
        for(GLuint j = 0; j < face.mNumIndices; j++)
            this->indices.push_back(face.mIndices[j]);
    }
}

void Mesh::updateModelMatrix()
{
    if(normalize)
    {
       float scaleFactor=1.0/maxDim;
        modelMatrix=glm::scale(modelMatrix,glm::vec3(scaleFactor));
    }
    if(moveToWorldCenter)
    {
        glm::vec3 translationVector(-centerOfMass);
        modelMatrix=glm::translate(modelMatrix,translationVector);
    }
}
//Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices)
//{
//    this->vertices=vertices;
//    this->indices=indices;

//    setupMesh();
//}

void Mesh::Draw()
{
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES,indices.size(),GL_UNSIGNED_INT,0);
    glBindVertexArray(0);
}

glm::mat4 Mesh::getModelMatrix() const
{
    return modelMatrix;
}

void Mesh::findCenterOfMass()
{
    glm::vec3 positionSum{0};
    for(const Vertex& vert:vertices)
        positionSum+=vert.Position;
    uint n=vertices.size();
    centerOfMass={positionSum.x/n,positionSum.y/n,positionSum.z/n};
}

void Mesh::findMaxDim()
{
    std::vector<Vertex>::iterator first,last;
    first=vertices.begin();
    last=vertices.end();
    if(first==last) maxDim=1;

    std::vector<Vertex>::iterator xmin,xmax,ymin,ymax,zmin,zmax;

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
}

void Mesh::setupMesh()
{
    glGenVertexArrays(1,&VAO);
    glGenBuffers(1,&VBO);
    glGenBuffers(1,&EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,vertices.size()*sizeof(Vertex),&vertices[0],GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,indices.size()*sizeof(GLuint),&indices[0],GL_STATIC_DRAW);

    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(GLvoid*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(GLvoid*)offsetof(Vertex,Normal));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

}

void Mesh::printMeshInformation() const
{
    std::cout<<"Number of verts:"<<vertices.size()<<std::endl;
    std::cout<<"Number of faces:"<<indices.size()<<std::endl;
}
