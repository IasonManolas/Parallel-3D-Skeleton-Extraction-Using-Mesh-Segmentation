#ifndef MESHSEGMENT_H
#define MESHSEGMENT_H

#include "cgaltypedefs.h"

class MeshSegment {
public:
  MeshSegment() {}

  CGALSurfaceMesh M;
  std::vector<CGALSurfaceMesh::Point> vertices;
  std::vector<std::vector<size_t>> indices;
  std::vector<size_t> vertexCorrespondence; // vertexCorrespondence[i] contains
                                            // the index of the vertex in the
                                            // whole mesh
};

#endif // MESHSEGMENT_H
