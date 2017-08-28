#ifndef SEGMENTCONTRACTOR_H
#define SEGMENTCONTRACTOR_H

#include <unordered_set>
#include <vector>

#include "cgaltypedefs.h"
#include <CGAL/Polygon_mesh_processing/Weights.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <Eigen/Eigen>
#include <boost/graph/graph_traits.hpp>
// Cotangent weight calculator
using Weight_calculator = CGAL::internal::Cotangent_weight<
    CGALSurfaceMesh,
    boost::property_map<CGALSurfaceMesh, boost::vertex_point_t>::type,
    CGAL::internal::Cotangent_value_minimum_zero<
        CGALSurfaceMesh,
        boost::property_map<CGALSurfaceMesh, boost::vertex_point_t>::type,
        CGAL::internal::Cotangent_value_Meyer_secure<CGALSurfaceMesh>>>;

using halfedge_descriptor =
    boost::graph_traits<CGALSurfaceMesh>::halfedge_descriptor;
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
class MeshContractor {
public:
  MeshContractor() {}
  MeshContractor(CGALSurfaceMesh meshToContract);
  CGALSurfaceMesh getContractedMesh() const;
  void contractMesh();

private:
  CGALSurfaceMesh m_M;
  double m_originalVolume;
  double m_volumeThreshold{std::pow(10, -6)};
  double m_Sl{2};
  Matrix m_Wh;
  Matrix m_Wl;
  Matrix m_L;
  Vector m_A0;
  Vector m_A;
  size_t m_maxNumOfIterations{10};
  size_t m_iterationsCompleted{0};

private:
  void calculateSkeleton();
  void computeLaplaceOperator();
  double computeAngleOppositeToEdge(CGALSurfaceMesh::Edge_index,
                                    size_t edgeSide) const;
  Matrix constructVertexMatrix() const;
  Matrix solveForNewVertexPositions(Matrix currentVertexPositions) const;
  void updateMeshPositions(Matrix Vnew);
  void updateWl();
  void updateWh();
  void computeOneRingAreaVector();
  double computeOneRingAreaAroundVertex(CGALSurfaceMesh::Vertex_index) const;
};

#endif // SEGMENTCONTRACTOR_H
