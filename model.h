#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

#include <SOIL.h>

#include "mesh.h"
#include "shader.h"
#include "material.h"

class Model
{
public:
    Model(){}
    Model(GLchar* path);
    void Draw();
    std::vector<Mesh> meshes;
    void setUniforms(Shader* shader) const
    {
        material.setUniforms(shader);
    }

private:
    Material material{Material(glm::vec3(0,0,0),glm::vec3(0.5,0.5,0),glm::vec3(0.6,0.6,0.5),128*0.25)};
    std::string directory{};

    void loadModel(std::string path);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh);
    void printModelInformation() const ;
};

#endif // MODEL_H
