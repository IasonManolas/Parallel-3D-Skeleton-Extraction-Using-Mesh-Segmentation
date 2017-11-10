#ifndef SKELETON_H
#define SKELETON_H


#include "cgaltypedefs.h"
#include "drawableskeleton.h"
#include <glm/gtx/string_cast.hpp>
#include <glm/mat4x4.hpp>
class Skeleton:public DrawableSkeleton {

private:
public:

  // public member functions
  // NOTE PointSphere should not be an argument
  // but for some reason PointSphere cannot be
  // loaded in this class but only through
  // Scene::load.Maybe because
  // GLWidget::makeCurrent() needs to be called?
  Skeleton(PointSphere &PS, glm::mat4 &modelMatrix)
      :DrawableSkeleton(PS,modelMatrix) {}
  // void setUniforms(Shader *shader);
  void clear() {
    m_drawingVector.clear();
    m_vertices.clear();
    m_indices.clear();
    updateMeshBuffers();
  }

  void initialize()
  {
     initializeDrawingBuffers();
  }

  void append(std::vector<std::vector<size_t>> newEdges,
              std::vector<CGALSurfaceMesh::Point> newNodePositions) {
    appendEdges(newEdges);
    appendNodes(newNodePositions);
    updateMeshBuffers();
    //std::cout << "New size of m_vertices is:" << m_vertices.size() << std::endl;
    //for (auto vertex : m_vertices)
    //  std::cout << glm::to_string(vertex) << std::endl;
    //std::cout << "New size of m_indices is:" << m_indices.size() << std::endl;
    //for (auto index : m_indices)
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
  void appendNodes(std::vector<CGALSurfaceMesh::Point> newNodePositions) {
    for (CGALSurfaceMesh::Point p : newNodePositions) {
      PointSphere tempPS = m_PS;
      tempPS.setPosition(p);
      tempPS.setColor(glm::vec3(1, 0, 0));
      m_drawingVector.push_back(tempPS);

      m_vertices.push_back(glm::vec3(p.x(), p.y(), p.z()));
    }
  }

};

#endif // SKELETON_H
