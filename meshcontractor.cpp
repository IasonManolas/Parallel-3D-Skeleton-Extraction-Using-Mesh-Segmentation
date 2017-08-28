#include "meshcontractor.h"

MeshContractor::MeshContractor(CGALSurfaceMesh meshToContract)
    : m_M(meshToContract) {
  m_M.collect_garbage();
  if (!CGAL::is_closed(m_M)) {
    using Halfedge_handle =
        boost::graph_traits<CGALSurfaceMesh>::halfedge_descriptor;
    using Facet_handle = boost::graph_traits<CGALSurfaceMesh>::face_descriptor;
    BOOST_FOREACH (Halfedge_handle h, halfedges(m_M)) {
      if (m_M.is_border(h)) {
        std::vector<Facet_handle> patch_facets;
        CGAL::Polygon_mesh_processing::triangulate_hole(
            m_M, h, std::back_inserter(patch_facets));
        // if (!success)
        //  std::cerr << "ERROR:Model is not closed and hole filling failed."
        //            << std::endl;
      }
    }
  }
  // NOTE teapot
  m_originalVolume = CGAL::Polygon_mesh_processing::volume(m_M);
  m_Wh = Matrix::Identity(m_M.number_of_vertices(),
                          m_M.number_of_vertices()); // ok
  double averageFaceArea =
      CGAL::Polygon_mesh_processing::area(m_M) / m_M.number_of_faces(); // ok
  m_Wl = Matrix::Identity(m_M.number_of_vertices(),
                          m_M.number_of_vertices()) *
         0.001 * std::sqrt(averageFaceArea); // ok
  m_L = Matrix::Zero(m_M.number_of_vertices(), m_M.number_of_vertices());
  computeLaplaceOperator();
  m_A = Vector::Zero(m_M.number_of_vertices());
  computeOneRingAreaVector();
  m_A0 = m_A;
  // calculateSkeleton();
}

void MeshContractor::calculateSkeleton() {
  while (CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume >
         m_volumeThreshold /*&&
         numberOfIterations < maxNumOfIterations*/) {
    std::cout << "contracting mesh.." << std::endl;
    contractMesh();
  }
}

void MeshContractor::contractMesh() {
  std::cout << "Contracting Mesh.." << std::endl;
  // std::cout << "constructing vertex matrix.." << std::endl;
  Matrix V = constructVertexMatrix();
  // std::cout << "Solving system.." << std::endl;
  V = solveForNewVertexPositions(V);
  // std::cout << "Updating mesh positions.." << std::endl;
  updateMeshPositions(V);
  // std::cout << "Updating Wl matrix.." << std::endl;
  updateWl();
  // std::cout << "Computing one ring area vector.." << std::endl;
  computeOneRingAreaVector();
  // std::cout << "Updating Wh matrix.." << std::endl;
  updateWh();
  // std::cout << "Computing laplace operator.." << std::endl;
  computeLaplaceOperator();

  m_iterationsCompleted++;
  std::cout << "Number of iterations completed:" << m_iterationsCompleted
            << std::endl;
  std::cout << "current volume/original volume="
            << CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume
            << std::endl;
}

Matrix MeshContractor::constructVertexMatrix() const {
  Matrix V(m_M.number_of_vertices(), 3);
  for (auto v : m_M.vertices()) {
    const CGALSurfaceMesh::Point p = m_M.point(v);
    V(size_t(v), 0) = p.x();
    V(size_t(v), 1) = p.y();
    V(size_t(v), 2) = p.z();
  }
  return V;
}

Matrix MeshContractor::solveForNewVertexPositions(
    Matrix currentVertexPositions) const {
  Matrix WlL = m_Wl * m_L;
  Matrix A(WlL.rows() + m_Wh.rows(), WlL.cols());
  A << WlL, m_Wh;
  Matrix Bupper = Matrix::Zero(WlL.rows(), 3);
  Matrix Blower = m_Wh * currentVertexPositions;
  Matrix B(Bupper.rows() + Blower.rows(), 3);
  B << Bupper, Blower;
  Matrix newVertexPositions =
      //   A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
      // A.colPivHouseholderQr().solve(B);
      (A.transpose() * A).ldlt().solve(A.transpose() * B);

  return newVertexPositions;
}

void MeshContractor::updateMeshPositions(Matrix Vnew) {
  int i = 0;
  for (auto v : m_M.vertices()) {
    // std::cout << "coords before updating:" << M.point(v) << std::endl;
    m_M.point(v) = CGALSurfaceMesh::Point(Vnew(i, 0), Vnew(i, 1), Vnew(i, 2));
    // std::cout << "coords after updating:" << M.point(v) << std::endl;
    i++;
  }
}

void MeshContractor::updateWl() { m_Wl *= m_Sl; }

void MeshContractor::updateWh() {
  for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
    m_Wh(i, i) = std::sqrt(m_A0(i) / m_A(i));
    // Wh(i, i) = Wh(i, i) * std::sqrt(A0(i) / A(i));
  }
}

void MeshContractor::computeOneRingAreaVector() {
  for (auto v : m_M.vertices()) {
    m_A(size_t(v)) = computeOneRingAreaAroundVertex(v);
  }
}

double MeshContractor::computeOneRingAreaAroundVertex(
    CGALSurfaceMesh::Vertex_index v) const {
  return CGAL::Polygon_mesh_processing::area(
      m_M.faces_around_target(m_M.halfedge(v)), m_M);
}

void MeshContractor::computeLaplaceOperator() {

  m_L = Matrix::Zero(m_M.number_of_vertices(), m_M.number_of_vertices());
  Weight_calculator m_weight_calculator(m_M);

  BOOST_FOREACH (halfedge_descriptor hd, halfedges(m_M)) {
    double weight = m_weight_calculator(hd);
    size_t i = size_t(m_M.source(hd));
    size_t j = size_t(m_M.target(hd));
    m_L(i, j) = 2 * weight; // multiplied with 2 because m_weight_calculator's
                            // operator() returns cota/2+cotb/2
  }

  // populate diagonal elements
  for (size_t row = 0; row < m_M.number_of_vertices(); row++) {
    double rowSum = 0;
    for (size_t col = 0; col < m_M.number_of_vertices(); col++) {
      rowSum -= m_L(row, col);
    }
    m_L(row, row) = rowSum;
  }
}
// double MeshContractor::computeAngleOppositeToEdge(CGALSurfaceMesh::Edge_index
// e,
//                                                  size_t edgeSide) const {
//  CGALSurfaceMesh::Halfedge_index halfedge = M.halfedge(e, edgeSide);
//  CGALSurfaceMesh::Halfedge_index nextHalfedge = M.next(halfedge);
//  CGALSurfaceMesh::Vertex_index vertexOppositeToEdge = M.target(nextHalfedge);
//  CGALSurfaceMesh::Point Pa = M.point(vertexOppositeToEdge);
//  CGALSurfaceMesh::Vertex_index edgeVertex0 = M.vertex(e, 0);
//  CGALSurfaceMesh::Point P0 = M.point(edgeVertex0);
//
//  CGALSurfaceMesh::Vertex_index edgeVertex1 = M.vertex(e, 1);
//  CGALSurfaceMesh::Point P1 = M.point(edgeVertex1);
//  CGAL::Vector_3<Kernel> v1 = P0 - Pa;
//  CGAL::Vector_3<Kernel> v2 = P1 - Pa;
//  double cosine = v1 * v2 / CGAL::sqrt(v1 * v1) / CGAL::sqrt(v2 * v2);
//  return std::acos(cosine); // IN RADIANS
//}

CGALSurfaceMesh MeshContractor::getContractedMesh() const { return m_M; }
