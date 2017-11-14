#ifndef SKELETON_H
#define SKELETON_H

#include "cgaltypedefs.h"
#include "drawableskeleton.h"
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
class Skeleton : public DrawableSkeleton {

public:
  // public member functions
  // NOTE PointSphere should not be an argument
  // but for some reason PointSphere cannot be
  // loaded in this class but only through
  // Scene::load.Maybe because
  // GLWidget::makeCurrent() needs to be called?
  Skeleton() : DrawableSkeleton() {}
  // void setUniforms(Shader *shader);
  void clear() {
    m_drawingVector.clear();
    m_vertices.clear();
    m_indices.clear();
    updateMeshBuffers();
  }

  const std::vector<glm::vec3> &getNodes() const { return m_vertices; }

  size_t getNumberOfNodes() const { return m_vertices.size(); }
  void initialize() { initializeDrawingBuffers(); }

  void append(std::vector<std::vector<size_t>> newEdges,
              std::vector<CGALSurfaceMesh::Point> newNodePositions,
              PointSphere psPrototype) {
    appendEdges(newEdges);
    appendNodes(newNodePositions, psPrototype);
    updateMeshBuffers();
    // std::cout << "New size of m_vertices is:" << m_vertices.size() <<
    // std::endl;
    // for (auto vertex : m_vertices)
    //  std::cout << glm::to_string(vertex) << std::endl;
    // std::cout << "New size of m_indices is:" << m_indices.size() <<
    // std::endl;
    // for (auto index : m_indices)
    //  std::cout << index << std::endl;
  }

  // private data members

  // private member functions
private:
  void appendEdges(std::vector<std::vector<size_t>> &newEdges) {

    for (std::vector<size_t> edge : newEdges) {
      m_indices.push_back(edge[0] + m_vertices.size());
      m_indices.push_back(edge[1] + m_vertices.size());
    }
  }
  void appendNodes(std::vector<CGALSurfaceMesh::Point> newNodePositions,
                   PointSphere psPrototype) {
    for (CGALSurfaceMesh::Point p : newNodePositions) {
      PointSphere tempPS = psPrototype;
      tempPS.setPosition(p);
      tempPS.setColor(glm::vec3(1, 0, 0));
      m_drawingVector.push_back(tempPS);

      m_vertices.push_back(glm::vec3(p.x(), p.y(), p.z()));
    }
  }
};

inline double glmSquaredDistance(glm::vec3 v1, glm::vec3 v2) {
  return std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2) +
         std::pow(v1.z - v2.z, 2);
}

inline std::pair<size_t, size_t> findClosestNodes(
    const Skeleton &s1,
    const Skeleton
        &s2) // returns pair(indexInFirstSkeleton,indexInSecondSkeleton)
{
  const std::vector<glm::vec3> &s1Nodes = s1.getNodes();
  const std::vector<glm::vec3> &s2Nodes = s2.getNodes();
  double minDistance = glmSquaredDistance(s1Nodes[0], s2Nodes[0]);
  std::pair<size_t, size_t> indicesOfMinDistance(0, 0);
  for (size_t index1 = 0; index1 < s1.getNumberOfNodes(); index1++) {
    glm::vec3 s1NodePos = s1Nodes[index1];
    for (size_t index2 = 0; index2 < s2.getNumberOfNodes(); index2++) {
      glm::vec3 s2NodePos = s2Nodes[index2];
      double d = glmSquaredDistance(s1NodePos, s2NodePos);
      if (d < minDistance) {
        minDistance = d;
        indicesOfMinDistance = std::pair<size_t, size_t>(index1, index2);
      }
    }
  }
  return indicesOfMinDistance;
}

#endif // SKELETON_H
