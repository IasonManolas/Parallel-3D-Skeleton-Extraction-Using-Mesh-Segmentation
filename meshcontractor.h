#ifndef SEGMENTCONTRACTOR_H
#define SEGMENTCONTRACTOR_H

//#include <algorithm>
#include <vector>

#include "cgaltypedefs.h"
#include <CGAL/Polygon_mesh_processing/Weights.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
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
using EigenMatrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using SpMatrix = Eigen::SparseMatrix<double>;
using DiagMatrix = Eigen::Diagonal<double>;
using EigenTriplet = Eigen::Triplet<double>;
static constexpr double maxNumber{500000};
class MeshContractor {

public:
  MeshContractor() {}
  MeshContractor(
      CGALSurfaceMesh meshToContract /*, std::vector<uint> indices*/);
  CGALSurfaceMesh getContractedMesh() const;
  void contractMesh();
  void executeContractionStep();
  void executeContractionReversingStep();
  void setVolumeThreshold(double volumeThreshold);

  //~~~~~~~~DEBUG~~~~~~
  size_t getMaxLToWhIndex() { return maxLtoWhIndex; }
  size_t getPreviousMaxLToWhIndex() { return previousMaxLtoWhIndex; }
  std::vector<size_t> getVerticesForWhichPreviousCotWeightWasUsed() {
    return problematicVertices;
  }
  std::vector<size_t> getLowOneRingAreaVertices() {
    return std::vector<size_t>{lowOneRingAreaVertices.begin(),
                               lowOneRingAreaVertices.end()};
  }
  std::vector<size_t> getHighOneRingAreaVertices() {
    return std::vector<size_t>{highOneRingAreaVertices.begin(),
                               highOneRingAreaVertices.end()};
  }
  void printFixedVertices(EigenMatrix verticesMatrix);
  std::vector<double> getLaplacianValues();

private:
  // CGALSurfaceMesh &m_M;
  CGALSurfaceMesh m_M;
  double m_originalVolume;
  static double
      m_volumeThreshold; // m_originalVolume/currentVolume least ratio so the
                         // contraction process stops
  double m_Sl{2};
  SpMatrix m_Wh;
  double m_Wh0{1.0};
  SpMatrix m_Wl;
  SpMatrix m_L;
  SpMatrix previousLaplaceOperator;
  EigenMatrix m_previousDeltaCoords;
  Vector m_A0;
  Vector m_A;
  Eigen::MatrixXi F;
  static constexpr size_t m_maxNumOfIterations{100};
  size_t m_iterationsCompleted{0};

  std::set<size_t> lowOneRingAreaVertices;
  std::set<size_t> highOneRingAreaVertices;
  std::vector<double> m_initialFaceAreas;
  std::vector<bool> isVertexFixed;

  std::vector<EigenMatrix> m_previousVertexPositions;

private:
  void computeLaplaceOperator();
  void computeFixedVertices();
  SpMatrix computeLaplaceOperatorUsingIGL();
  double computeAngleOppositeToEdge(CGALSurfaceMesh::Edge_index,
                                    size_t edgeSide) const;
  EigenMatrix constructVertexMatrix() const;
  EigenMatrix solveForNewVertexPositions(EigenMatrix currentVertexPositions);
  void updateMeshPositions(EigenMatrix Vnew);
  void detectDegeneracies();
  void updateWl();
  void updateWh();
  void computeOneRingAreaVector();
  double computeOneRingAreaAroundVertex(CGALSurfaceMesh::Vertex_index) const;
  boost::optional<double>
  computeCotangentWeight(CGALSurfaceMesh::edge_index ei) const;
  boost::optional<double>
  computeCotangentWeight(CGALSurfaceMesh::halfedge_index hei) const;
  boost::optional<double> computeCotangentValue(Kernel::Vector_3,
                                                Kernel::Vector_3) const;
  int maxLtoWhIndex{-1};
  int previousMaxLtoWhIndex{-1};
  std::vector<size_t> problematicVertices;
};

#endif // SEGMENTCONTRACTOR_H
