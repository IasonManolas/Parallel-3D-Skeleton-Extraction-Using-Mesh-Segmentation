#ifndef POLYGONALMESH_H
#define POLYGONALMESH_H

#include "cgaltypedefs.h"
#include <vector>

class PolygonalMesh {
protected:
  std::vector<MyVertex> m_vertices;
  std::vector<uint> m_indices;
  CGALSurfaceMesh m_M;             // In model coordinates
  glm::vec3 centerOfMass{0, 0, 0}; // in model coordinates
  float maxDim{1.0};               // in model coordinates
  double averageEdgeLength{1};

  void createCGALSurfaceMesh() {
    m_M.clear();
    std::cout << "Building CGALPolygonMesh.." << std::endl;
    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<std::size_t>> polygons;

    for (auto const &vert : m_vertices) {
      points.push_back(
          Kernel::Point_3(vert.Position.x, vert.Position.y, vert.Position.z));
      // Kernel::Point_3((vert.Position.x - centerOfMass.x) / maxDim,
      //
      //                (vert.Position.y - centerOfMass.y) / maxDim,
      //                (vert.Position.z - centerOfMass.z) / maxDim));
    }
    for (size_t i = 0; i < m_indices.size(); i += 3) {
      std::vector<std::size_t> tri{m_indices[i], m_indices[i + 1],
                                   m_indices[i + 2]};
      polygons.push_back(tri);
    }
    //               CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
    // std::cout << "is polygon soup polygon mesh:"
    //          <<
    //          CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
    //                 polygons)
    //          << std::endl;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,
                                                                polygons, m_M);

    // std::cout << "Finished building Polygon Mesh." << std::endl;
    // std::cout << "Is closed:" << CGAL::is_closed(m_M) << std::endl;
  }
  void createCGALSurfaceMesh(std::string filename) {
    std::ifstream inputFile(filename);
    assert(inputFile);
    m_M.clear();
    CGAL::read_off(inputFile, m_M);
  }

  void updateDrawingVertices() { // uses M to update std::vector<MyVertex>
    // m_vertices
    for (auto v : m_M.vertices()) {
      CGALSurfaceMesh::Point p = m_M.point(v);
      m_vertices[size_t(v)].Position = glm::vec3(p.x(), p.y(), p.z());
    }
  }

public:
  void printMeshInformation() const {
    std::cout << "Number of m_vertices:" << m_vertices.size() << std::endl;
    std::cout << "Number of faces:" << m_indices.size() / 3 << std::endl;
  }

  std::vector<MyVertex> getVertices() const { return m_vertices; }
  CGALSurfaceMesh getM() const { return m_M; }
  std::vector<uint> getIndices() const { return m_indices; }
};

#endif // POLYGONALMESH_H
