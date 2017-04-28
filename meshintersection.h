#ifndef MESHINTERSECTION_H
#define MESHINTERSECTION_H

#include "cgaltypedefs.h"

namespace MeshIntersection {

inline Ray_intersection MeshRayIntersection(CGALSurfaceMesh M,
                                            Kernel::Ray_3 ray) {
  Tree tree(faces(M).first, faces(M).second, M);

  boost::optional<int> a = 3;

  std::cout << "Computing intersection" << std::endl;
  Ray_intersection intersection = tree.first_intersection(ray);
  return intersection;
  //    std::cout << "Computed intersection" << std::endl;
  //    if (intersection) {
  //      std::cout << "Intersection(s) found!" << std::endl;
  //      if (boost::get<Point>(&(intersection->first))) {

  //        return true;
  //      }
  //    }
  //    std::cout << "No Intersection found." << std::endl;
  //    return false;
}

inline int
findClosestVertex(CGALSurfaceMesh M, Point intersectionPoint,
                  CGALSurfaceMesh::Face_index intersectingFaceIndex) {
  CGALSurfaceMesh::halfedge_index beginHalfedge =
      M.halfedge(intersectingFaceIndex);
  CGALSurfaceMesh::halfedge_index h = beginHalfedge;
  CGALSurfaceMesh::vertex_index vIndex = M.source(h);
  Kernel::Point_3 vertex = M.point(vIndex);
  double minDistance = CGAL::squared_distance(vertex, intersectionPoint);
  int closestVertexIndex = (int)vIndex;
  //       std::cout << "vertices around vertex " <<v<< std::endl;
  do {
    std::cout << h << std::endl;
    vIndex = M.source(h);
    Kernel::Point_3 vertex = M.point(vIndex);
    double distance = CGAL::squared_distance(vertex, intersectionPoint);
    if (distance < minDistance) {

      minDistance = distance;
      vIndex = M.source(h);
      closestVertexIndex = int(vIndex);
    }
    h = M.next(h);
  } while (h != beginHalfedge);
  return closestVertexIndex;
}
}
#endif // MESHINTERSECTION_H
