#ifndef SKELETON_H
#define SKELETON_H

#include <boost/graph/adjacency_list.hpp>

#include "cgaltypedefs.h"
#include "drawableskeleton.h"
#include "pointsphere.h"
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
class Skeleton {

public:
  std::vector<glm::vec3> m_vertices;
  std::vector<uint> m_indices;

  uint VAO, VBO, EBO;

  void drawEdges() {
    glBindVertexArray(VAO);
    glDrawElements(GL_LINES, m_indices.size(), GL_UNSIGNED_INT, 0);
    // glDrawArrays(GL_LINES, 0, m_vertices.size());
    glBindVertexArray(0);
  }
  // public member functions
  // NOTE PointSphere should not be an argument
  // but for some reason PointSphere cannot be
  // loaded in this class but only through
  // Scene::load.Maybe because
  // GLWidget::makeCurrent() needs to be called?
  Skeleton(PointSphere &PS, glm::mat4 &modelMatrix)
      : m_PS(PS), m_meshModelMatrix(modelMatrix) {}
  // void setUniforms(Shader *shader);
  void Draw(Shader *skeletonShader, Shader *shader) {
    skeletonShader->Use();
    drawEdges();
    shader->Use();
    drawNodes(shader);
  }
  void clear() {
    m_drawingVector.clear();
    m_vertices.clear();
    m_indices.clear();
    updateMeshBuffers();
  }

  void append(std::vector<std::vector<size_t>> newEdges,
              std::vector<CGALSurfaceMesh::Point> newNodePositions) {
    appendEdges(newEdges);
    appendNodes(newNodePositions);
    updateMeshBuffers();
    std::cout << "New size of m_vertices is:" << m_vertices.size() << std::endl;
    for (auto vertex : m_vertices)
      std::cout << glm::to_string(vertex) << std::endl;
    std::cout << "New size of m_indices is:" << m_indices.size() << std::endl;
    for (auto index : m_indices)
      std::cout << index << std::endl;
  }

  void updateMeshBuffers() {

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (m_vertices.size() + 1) * sizeof(glm::vec3),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_DYNAMIC_DRAW);

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
  void initializeDrawingBuffers() {

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (m_vertices.size() + 1) * sizeof(glm::vec3),
                 &m_vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(GLuint),
                 &m_indices[0], GL_DYNAMIC_DRAW);

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
  // private data members
private:
  const PointSphere &m_PS;
  const glm::mat4 &m_meshModelMatrix;
  std::vector<PointSphere> m_drawingVector;

  // private member functions
private:
  void appendEdges(std::vector<std::vector<size_t>> &newEdges) {

    for (std::vector<size_t> edge : newEdges) {
      m_indices.push_back(edge[0] + m_vertices.size());
      m_indices.push_back(edge[1] + m_vertices.size());
    }
  }
  void appendNodes(std::vector<CGALSurfaceMesh::Point> newNodePositions) {
    for (CGALSurfaceMesh::Point p : newNodePositions) {
      PointSphere tempPS = m_PS;
      tempPS.setPosition(p);
      tempPS.setColor(glm::vec3(1, 0, 0));
      m_drawingVector.push_back(tempPS);

      m_vertices.push_back(glm::vec3(p.x(), p.y(), p.z()));
    }
  }

  void drawNodes(Shader *shader) {
    shader->Use();
    for (PointSphere ps : m_drawingVector) {
      ps.handle_drawing(shader, m_meshModelMatrix);
    }
  }
};

#endif // SKELETON_H
