#ifndef DRAWABLEMESH_H
#define DRAWABLEMESH_H

#include "material.h"
#include "polygonalmesh.h"
#include "shader.h"
class DrawableMesh : public PolygonalMesh {
public:
  DrawableMesh() {
    for (auto &color : colorPalette) {
      color /= 255.0;
    }
  }
  void drawMesh() {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }
  void setMaterial(const Material &value) { material = value; }

  glm::mat4 getModelMatrix() const { return m_modelMatrix; }

protected:
  uint VAO, VBO, EBO;
  Material material{glm::vec3(0.05, 0.05, 0.05),
                    glm::vec3(1.0, 0.75, 0.5), // should be static constexpr
                    glm::vec3(0.1, 0.075, 0.05), 80 * 0.85};
  double alphaValue{1.0f};
  glm::mat4 m_modelMatrix{1.0}; //  modelSpace->worldSpace
  std::vector<glm::vec3> colorPalette{
      // this gets normalized in the constructor!
      // NOTE this should be static
      glm::vec3(128.0, 0, 0),   glm::vec3(255, 0, 0),
      glm::vec3(255, 200, 220), glm::vec3(170.0, 110, 40),
      glm::vec3(255.0, 150, 0), glm::vec3(255.0, 215.0, 180),
      glm::vec3(128, 128, 0),   glm::vec3(255, 235, 0),
      glm::vec3(255, 250, 200), glm::vec3(190, 255, 0),
      glm::vec3(0, 190, 0),     glm::vec3(170, 255, 195),
      glm::vec3(0, 128, 128),   glm::vec3(100, 255, 255),
      glm::vec3(0, 0, 128),     glm::vec3(67, 133, 255),
      glm::vec3(130, 0, 150),   glm::vec3(230, 190, 255),
      glm::vec3(255, 0, 255),   glm::vec3(128, 128, 128)};

protected:
  virtual void setUniforms(Shader *shader) = 0;
  void updateMeshBuffers() {

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(MyVertex),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)offsetof(MyVertex, Normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)offsetof(MyVertex, Color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
  }
  void initializeDrawingBuffers() {

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(MyVertex),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)offsetof(MyVertex, Normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                          (GLvoid *)offsetof(MyVertex, Color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    // std::cout << "printDebugInformation was called in "<<__func__<<std::endl;
    //     // printDebugInformation();
    //
  }

  void printDebugInformation() const {
    glBindVertexArray(VAO);
    for (int i = 0; i <= 1; i++) {
      GLint ival;
      GLvoid *pval;

      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &ival);
      printf("Attr %d: ENABLED    		= %d\n", i, ival);
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &ival);
      printf("Attr %d: SIZE		= %d\n", i, ival);
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &ival);
      printf("Attr %d: STRIDE		= %d\n", i, ival);
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &ival);
      printf("Attr %d: TYPE		= 0x%x\n", i, ival);
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, &ival);
      printf("Attr %d: NORMALIZED		= %d\n", i, ival);
      glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &ival);
      printf("Attr %d: BUFFER		= %d\n", i, ival);
      glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pval);
      printf("Attr %d: POINTER		= %d\n", i, ival);
    }
    // Also print the numeric handle of the VAO:
    printf("VAO = %ld\n", long(VAO));
  }
};

#endif // DRAWABLEMESH_H
