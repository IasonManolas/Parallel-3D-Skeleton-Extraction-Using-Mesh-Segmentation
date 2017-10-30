#ifndef DRAWABLESKELETON_H
#define DRAWABLESKELETON_H

#include "shader.h"
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <vector>

class DrawableSkeleton {

public:
  DrawableSkeleton() {
    m_vertices.push_back(glm::vec3(-1, 0, 0));
    m_vertices.push_back(glm::vec3(1, 0, 0));

    m_indices.push_back(0);
    m_indices.push_back(1);
  }

  void drawEdges() {
    glBindVertexArray(VAO);
    glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0);
    // if (!m_vertices.empty())
    //  for (auto vertex : m_vertices)
    //    std::cout << glm::to_string(vertex) << std::endl;

    glBindVertexArray(0);
  }
  // void setMaterial(const Material &value) { material = value; }
private:
protected:
  std::vector<glm::vec3> m_vertices;
  std::vector<uint> m_indices;
  uint VAO, VBO, EBO;
  // Material material{glm::vec3(0.05, 0.05, 0.05),
  //                  glm::vec3(0.5, 0.5, 0), // should be static constexpr
  //                  glm::vec3(0.6, 0.6, 0.5), 128 * 0.25};
protected:
  void setUniforms() {}

  void updateMeshBuffers() {

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(glm::vec3),
                 &m_vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
  }
  void initializeDrawingBuffers() {

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(glm::vec3),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);

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
#endif // DRAWABLESKELETON_H
