#ifndef POINTSPHERE_H
#define POINTSPHERE_H

#include <string>

#include "material.h"
#include "mesh.h"
class PointSphere : public Mesh {

public:
  PointSphere() {}

  void updateRadius(const CGALSurfaceMesh &onWhichMesh) {
    m_desiredRadius = m_radius_averageEdge_proportion *
                      meshMeasuring::findAverageEdgeLength(onWhichMesh);
  }

  // loads a model and sets the sphere's size
  void load(std::string filename) {
    resetMeshAttributes();
    std::tie(indices, vertices) =
        meshLoader::load(filename); // vertices contains coords & normals
    setupDrawingBuffers();

    // Find the model matrix that normalizes the mesh
    centerOfMass = meshMeasuring::findCenterOfMass(vertices);
    maxDim = meshMeasuring::findMaxDimension(vertices);
    updateModelMatrix();

    // buildPolygonMesh();
    // printMeshInformation();
  }
  void setPosition(double x, double y, double z) {
    CGALSurfaceMesh::Point p(x, y, z);
    m_position = p;
    updateModelMatrix();
  }

  void setPosition(CGALSurfaceMesh::Point newPosition) {
    m_position = newPosition;
    updateModelMatrix();
  }
  CGAL::Point_3<CGAL::Exact_predicates_inexact_constructions_kernel>
  getPosition() {
    return m_position;
  }
  void Draw(Shader *shader) {
    setUniforms(shader);
    Mesh::Draw();
  }

private:
  double m_desiredRadius;
  double m_radius_averageEdge_proportion{0.05};
  CGALSurfaceMesh::Point m_position;
  Material m_material{Material(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0),
                               glm::vec3(1, 0, 0), 128 * 0.25)};

  void updateModelMatrix() // Model space -> World space
  {
    modelMatrix = glm::mat4{1.0};
    glm::vec3 translationVector(-centerOfMass + glm::vec3(m_position.x(),
                                                          m_position.y(),
                                                          m_position.z()));
    modelMatrix = glm::translate(modelMatrix, translationVector);

    float scaleFactor =
        m_desiredRadius /
        std::sqrt(pow(centerOfMass.x - vertices[0].Position.x, 2) +
                  pow(centerOfMass.y - vertices[0].Position.y, 2) +
                  pow(centerOfMass.z - vertices[0].Position.z, 2));
    modelMatrix = glm::scale(modelMatrix, glm::vec3(scaleFactor));
  }
  void setUniforms(Shader *shader) {
    m_material.setUniforms(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1,
                       GL_FALSE, glm::value_ptr(modelMatrix));
  }
};
#endif
