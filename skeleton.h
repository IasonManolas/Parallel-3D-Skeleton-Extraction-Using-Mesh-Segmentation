#ifndef SKELETON_H
#define SKELETON_H

#include <boost/graph/adjacency_list.hpp>

#include "cgaltypedefs.h"
#include "pointsphere.h"

class Skeleton {

  // public member functions
public:
  Skeleton();
  Skeleton(const CGALSurfaceMesh &contractedMesh,
           const PointSphere &); // NOTE PointSphere should not be an argument
                                 // but for some reason PointSphere cannot be
                                 // loaded in this class but only through
                                 // Scene::load.Maybe because
                                 // GLWidget::makeCurrent() need to be called?
  // void setUniforms(Shader *shader);
  void Draw(Shader *shader);
  // private data members
private:
  std::vector<CGALSurfaceMesh::Point>
      m_skeletonPoints; // NOTE a vector might not be sufficient in the future
  std::vector<PointSphere> m_drawingVector;
  // PointSphere drawingSphere;

  // private member functions
private:
  void extract_skeleton_vertices(const CGALSurfaceMesh &M);
  void construct_drawing_spheres(const CGALSurfaceMesh &M, const PointSphere &);
};

#endif // SKELETON_H
