#ifndef MESH_H
#define MESH_H

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <assimp/types.h>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

#include "shader.h"

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
};

class Mesh
{
public:
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;

    Mesh(const std::vector<Vertex>& vertices,const std::vector<GLuint>& indices);
    void Draw(Shader *shader);

private:
    GLuint VAO,VBO,EBO;
    void setupMesh();
};

#endif // MESH_H
