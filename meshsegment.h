#ifndef MESHSEGMENT_H
#define MESHSEGMENT_H

#include "cgaltypedefs.h"
#include "drawablemesh.h"
#include "material.h"
#include <glm/mat4x4.hpp>

class MeshSegment : public DrawableMesh {
public:
  MeshSegment() {}
  MeshSegment(bool nothing) {
    m_vertices.resize(5, MyVertex());
    m_indices.resize(5, 0);
    initializeDrawingBuffers();
  }
  MeshSegment(const CGALSurfaceMesh &M, std::vector<MyVertex> wholeMeshVertices,
              const Facet_int_map &segmentMap, size_t segmentIndex,
              glm::mat4 modelMatrix)
      : m_modelMatrix(modelMatrix) {
    populate(M, wholeMeshVertices, segmentMap, segmentIndex);
    initializeDrawingBuffers();
  }
  void handle_drawing(Shader *shader) {
    shader->Use();
    setUniforms(shader);
    drawMesh();
  }

  // std::vector<CGALSurfaceMesh::Point> points;
  std::vector<size_t> vertexCorrespondence; // vertexCorrespondence[i] contains
                                            // the index of the vertex in the
                                            // whole mesh

  CGALSurfaceMesh M() const { return m_M; }

  void setSegment(MeshSegment s) {
    m_M = s.M();
    m_vertices = s.getVertices();
    m_indices = s.getIndices();
  }
  void setM(const CGALSurfaceMesh &M) {
    m_M = M;
    updateDrawingVertices();
    updateMeshBuffers();
  }

private:
  Material material{glm::vec3(0.05, 0.05, 0.05),
                    glm::vec3(1.0, 1.0, 1.0), // should be static constexpr
                    glm::vec3(1.0, 1.0, 1.0), 128 * 0.25};
  glm::mat4 m_modelMatrix;
  std::vector<std::vector<size_t>>
  createCGALSurfaceMesh(CGALSurfaceMesh M, Facet_int_map segmentMap,
                        size_t segmentIndex) {
    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<size_t>> faces;
    for (CGALSurfaceMesh::Face_iterator it = M.faces_begin();
         it != M.faces_end(); it++) {
      CGALSurfaceMesh::Face_index fIndex(*it);
      if (segmentMap[fIndex] == segmentIndex) {
        CGALSurfaceMesh::Halfedge_index h = M.halfedge(fIndex);
        // CGALSurfaceMesh::Halfedge_index hNext = M.next_around_target(h);
        // CGALSurfaceMesh::Halfedge_index hPrevious = M.next_around_source(h);

        // CGALSurfaceMesh::Edge_index e1 = M.edge(h);
        // CGALSurfaceMesh::Edge_index e2 = M.edge(hNext);
        // CGALSurfaceMesh::Edge_index e3 = M.edge(hPrevious);

        CGALSurfaceMesh::Vertex_index v1; //= M.source(h);
        CGALSurfaceMesh::Vertex_index v2; //= M.target(h);
        CGALSurfaceMesh::Vertex_index v3; //= M.target(hNext);

        auto vri = M.vertices_around_face(h);
        v1 = *vri.begin();
        v2 = *(++vri.begin());
        v3 = *(++++vri.begin());

        CGALSurfaceMesh::Point p1(M.point(v1));
        CGALSurfaceMesh::Point p2(M.point(v2));
        CGALSurfaceMesh::Point p3(M.point(v3));

        size_t i1 = std::distance(points.begin(),
                                  std::find(points.begin(), points.end(), p1));
        if (i1 == points.size()) {
          points.push_back(p1);
          vertexCorrespondence.push_back(size_t(v1));
          i1 = points.size() - 1;
        }

        size_t i2 = std::distance(points.begin(),
                                  std::find(points.begin(), points.end(), p2));
        if (i2 == points.size()) {
          points.push_back(p2);
          vertexCorrespondence.push_back(size_t(v2));
          i2 = points.size() - 1;
        }

        size_t i3 = std::distance(points.begin(),
                                  std::find(points.begin(), points.end(), p3));
        if (i3 == points.size()) {
          points.push_back(p3);
          vertexCorrespondence.push_back(size_t(v3));
          i3 = points.size() - 1;
        }

        std::vector<size_t> face{i1, i2, i3};
        faces.push_back(face);
      }
    }
    // points= points;
    std::cout << "is polygon soup polygon mesh:"
              << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
                     faces)
              << std::endl;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, faces,
                                                                m_M);
    std::cout << "Is closed:" << CGAL::is_closed(m_M) << std::endl;
    if (!CGAL::is_closed(m_M)) {
      std::cout << "Segment is not closed." << std::endl;
      std::cout << "Stitching mesh.." << std::endl;
      CGAL::Polygon_mesh_processing::stitch_borders(m_M);
    }
    std::cout << "Finished building Polygon Mesh." << std::endl;
    std::cout << "Is closed:" << CGAL::is_closed(m_M) << std::endl;
    return faces;
  }

  void populate(const CGALSurfaceMesh &M,
                std::vector<MyVertex> wholeMeshVertices,
                const Facet_int_map &segmentMap, size_t segmentIndex) {

    std::vector<std::vector<size_t>> faces =
        createCGALSurfaceMesh(M, segmentMap, segmentIndex);
    populateVertices(wholeMeshVertices);
    populateIndices(faces);
  }

  void populateVertices(std::vector<MyVertex> wholeMeshVertices) {
    m_vertices.resize(m_M.number_of_vertices());
    for (size_t i = 0; i < m_vertices.size(); i++) {
      MyVertex &vertex = m_vertices[i];
      vertex.Position = wholeMeshVertices[vertexCorrespondence[i]].Position;
      vertex.Normal = wholeMeshVertices[vertexCorrespondence[i]].Normal;
      vertex.Color = glm::vec3(1.0, 0.0, 0.0);
      // std::cout << "Vertex " << i
      //          << " Position:" << glm::to_string(mvertices[i].Position)
      //          << " Normal:" << glm::to_string(vertices[i].Normal)
      //          << " Color:" << glm::to_string(vertices[i].Color) <<
      //          std::endl;
    }
  }

  void populateIndices(std::vector<std::vector<size_t>> faces) {
    for (auto face : faces)
      for (auto vindex : face)
        m_indices.push_back(vindex);
  }

  void setUniforms(Shader *shader) override {
    shader->Use();
    material.setUniforms(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1,
                       GL_FALSE, glm::value_ptr(m_modelMatrix));
    // glUniform1f(glGetUniformLocation(shader->programID, "alpha"),
    // alphaValue);
  }
};

#endif // MESHSEGMENT_H
