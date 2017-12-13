#ifndef LINESEGMENTS_H
#define LINESEGMENTS_H

#include "shader.h"
#include <glm/vec3.hpp>
#include <vector>

class LineSegments {
public:
  void Draw(Shader *edgeShader) const {
    edgeShader->Use();
    drawEdges();
  }
  LineSegments() {}
  void clear() { m_vertices.clear(); }

  void add_edge(glm::vec3 p1, glm::vec3 p2) {
    m_vertices.push_back(p1);
    m_vertices.push_back(p2);
    updateMeshBuffers();
  }

  void initializeDrawingBuffers() {

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    // glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (m_vertices.size() + 1) * sizeof(glm::vec3),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
    //             &m_indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)offsetof(glm::vec3, y));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)offsetof(glm::vec3, z));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    // std::cout << "printDebugInformation was called in "<<__func__<<std::endl;
    //     // printDebugInformation();
    //
  }

private:
  void drawEdges() const {
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, m_vertices.size());
    glBindVertexArray(0);
  }
  std::vector<glm::vec3> m_vertices;

  uint VAO, VBO;
  // Material material{glm::vec3(0.05, 0.05, 0.05),
  //                  glm::vec3(0.5, 0.5, 0), // should be static constexpr
  //                  glm::vec3(0.6, 0.6, 0.5), 128 * 0.25};
private:
  void updateMeshBuffers() {

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (m_vertices.size() + 1) * sizeof(glm::vec3),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
    //             &m_indices[0], GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)offsetof(glm::vec3, y));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (GLvoid *)offsetof(glm::vec3, z));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
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

#endif // LINESEGMENTS_H
