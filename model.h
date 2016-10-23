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

class Model
{
public:
    Model(GLchar* path);
    void Draw(Shader* shader);
    std::vector<Mesh> meshes;
private:
    std::string directory;

    void loadModel(std::string path);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh,const aiScene* scene);
    const void getVertNumb();
};

#endif // MODEL_H
