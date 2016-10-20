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

GLint TextureFromFile(const char* path, std::string directory);

class Model
{
public:
    Model(GLchar* path);
    void Draw(Shader* shader);
private:
    std::vector<Mesh> meshes;
    std::string directory;
    std::vector<Texture> textures_loaded;	// Stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.

    void loadModel(std::string path);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh,const aiScene* scene);
    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type,
                                         std::string typeName);
    const void getVertNumb();
};

#endif // MODEL_H
