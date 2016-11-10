#ifndef MESH_H
#define MESH_H

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iterator>
#include <tuple>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shader.h"
#include "material.h"
#include "model.h"

struct MyVertex {
    glm::vec3 Position;
    glm::vec3 Normal;

};

class Mesh
{
public:
    std::vector<MyVertex> vertices;
    std::vector<GLuint> indices;

    Mesh(){}
    Mesh(std::string directory)
    {
         loadMesh(directory);
    findMeshInformationToNormalize();
    setupDrawingBuffers();
    printMeshInformation() ;

    }
    void Draw();
    void setUniforms(Shader* shader) const
    {
        material.setUniforms(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
    }


    glm::mat4 getModelMatrix() const;

    void loadMesh(std::string directory);
    void findMeshInformationToNormalize();
    void setupDrawingBuffers();
    void printMeshInformation() const ;
private:
    GLuint VAO,VBO,EBO;
    std::string directory{};
    Material material{Material(glm::vec3(0,0,0),glm::vec3(0.5,0.5,0),glm::vec3(0.6,0.6,0.5),128*0.25)};
    glm::mat4 modelMatrix{1.0};
    float maxDim;
    glm::vec3 centerOfMass;

    bool normalize{true};
    bool moveToWorldCenter{true};

    void findCenterOfMass();
    void findMaxDim();

    void clearVectors();
    void loadMeshAssimp(std::string path);
    void processNodeAssimp(aiNode* node, const aiScene* scene);
    void processMeshAssimp(aiMesh* mesh);

    void updateModelMatrix();

};

#endif // MESH_H
