#include "skeleton.h"

Skeleton::Skeleton() {}

Skeleton::Skeleton(const CGALSurfaceMesh &contractedMesh,
                   const PointSphere &ps) {
  m_skeletonPoints.clear();
  extract_skeleton_vertices(contractedMesh);
  construct_drawing_spheres(contractedMesh, ps);
}

void Skeleton::extract_skeleton_vertices(const CGALSurfaceMesh &M) {
  for (auto v : M.vertices()) {
    m_skeletonPoints.push_back(M.point(v));
  }
}

void Skeleton::construct_drawing_spheres(const CGALSurfaceMesh &M,
                                         const PointSphere &ps) {
  // PointSphere tempPS = ps;
  // tempPS.setPosition(CGALSurfaceMesh::Point(0, 0, 2));
  // m_drawingVector.push_back(tempPS);

  size_t index = 0;
  for (auto v : m_skeletonPoints) {
    PointSphere tempPS = ps;
    tempPS.setPosition(m_skeletonPoints[index]);
    index++;
    m_drawingVector.push_back(tempPS);
  }
}

// void Skeleton::setUniforms() {
//  // drawingSphere.setUniforms(shader);
//  // m_drawingVector[0].setUniforms(shader);
//  for (auto ps : m_drawingVector) {
//  }
//}

void Skeleton::Draw(Shader *shader) {
  // drawingSphere.Draw();
  // m_drawingVector[0].Draw();
  for (auto ps : m_drawingVector) {
    // ps.Draw(shader);
  }
}
