#include "mesh.h"

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices, const std::vector<Texture>& textures)
{
    this->vertices=vertices;
    this->indices=indices;
    this->textures=textures;

    setupMesh();
}

void Mesh::Draw(Shader* shader)
{
    GLuint diffuseNr=1;
    GLuint specularNr=1;

    for(GLuint i=0;i<textures.size();i++)
    {
        glActiveTexture(GL_TEXTURE0+i);
        std::stringstream ss;
        std::string number;
        std::string name=textures[i].type;
        if(name=="texture_diffuse") ss<<diffuseNr++;
        else if(name=="texture_specular") ss<<specularNr;
        number=ss.str();

        glUniform1f(glGetUniformLocation(shader->programID,("material."+name+number).c_str()),i);
        glBindTexture(GL_TEXTURE_2D,textures[i].id);
    }
    glActiveTexture(GL_TEXTURE0);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES,indices.size(),GL_UNSIGNED_INT,0);
    glBindVertexArray(0);
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

    glVertexAttribPointer(2,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(GLvoid*)offsetof(Vertex,TexCoords));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

}
