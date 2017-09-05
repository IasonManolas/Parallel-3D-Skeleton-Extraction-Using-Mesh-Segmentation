#include "meshcontractor.h"

MeshContractor::MeshContractor(CGALSurfaceMesh meshToContract)
    : m_M(meshToContract) {

  Eigen::initParallel();
  Eigen::setNbThreads(6);
  // int nthreads = Eigen::nbThreads();
  // std::cout << "THREADS = " << nthreads << std::endl; // returns '1'

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
      }
    }
  }
  // NOTE teapot
  m_originalVolume = CGAL::Polygon_mesh_processing::volume(m_M);
  m_Wh = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
  m_Wh.setIdentity();
  double averageFaceArea =
      CGAL::Polygon_mesh_processing::area(m_M) / m_M.number_of_faces(); // ok
  m_Wl = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
  m_Wl.setIdentity();
  m_Wl = m_Wl * 0.001 * std::sqrt(averageFaceArea); // ok
  computeLaplaceOperator();
  m_A = Vector::Zero(m_M.number_of_vertices());
  computeOneRingAreaVector();
  m_A0 = m_A;
  // calculateSkeleton();
}

void MeshContractor::calculateSkeleton() {
  while (CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume >
             m_volumeThreshold &&
         m_iterationsCompleted < m_maxNumOfIterations) {
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
SpMatrix concatenateVertically(SpMatrix A, SpMatrix B) {
  // A
  //---
  // B
  assert(A.cols() == B.cols());
  std::vector<Triplet> tripletVector;
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SpMatrix::InnerIterator it(A, k); it; ++it) {
      tripletVector.push_back(Triplet(it.row(), it.col(), it.value()));
    }
    for (SpMatrix::InnerIterator it(B, k); it; ++it) {
      tripletVector.push_back(
          Triplet(it.row() + A.rows(), it.col(), it.value()));
    }
  }
  SpMatrix M(A.rows() + B.rows(), A.cols());
  M.setFromTriplets(tripletVector.begin(), tripletVector.end());

  return M;
}

//#include <ctime>
Matrix MeshContractor::solveForNewVertexPositions(
    Matrix currentVertexPositions) const {
  SpMatrix WlL = m_Wl * m_L;
  SpMatrix A = concatenateVertically(WlL, m_Wh);
  // SpMatrix A(WlL.rows() + m_Wh.rows(), WlL.cols());
  // A << WlL, m_Wh;
  Matrix Bupper = Matrix::Zero(WlL.rows(), 3);
  Matrix Blower = m_Wh * currentVertexPositions;
  Matrix B(Bupper.rows() + Blower.rows(), 3);
  B << Bupper, Blower;

  // std::clock_t start;

  // start = std::clock();

  Eigen::SimplicialLDLT<SpMatrix> solver;
  solver.compute(A.transpose() * A);
  Matrix newVertexPositions = solver.solve(A.transpose() * B);
  // std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC)
  //          << " s" << std::endl;

  // std::cout << "#iterations to solve system:     " << solver.iterations()
  //          << std::endl;
  // std::cout << "estimated error: " << solver.error() << std::endl;
  return newVertexPositions;
}
void MeshContractor::updateMeshPositions(Matrix Vnew) {
  int i = 0;
  for (auto v : m_M.vertices()) {
    m_M.point(v) = CGALSurfaceMesh::Point(Vnew(i, 0), Vnew(i, 1), Vnew(i, 2));
    i++;
  }
}

void MeshContractor::updateWl() { m_Wl *= m_Sl; }

void MeshContractor::updateWh() {
  for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
    m_Wh.coeffRef(i, i) = std::sqrt(m_A0(i) / m_A(i));
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

  m_L = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());

  std::vector<Triplet> tripletVector;
  std::vector<double> diagonalElements(m_M.number_of_vertices(), 0);
  Weight_calculator m_weight_calculator(m_M);

  BOOST_FOREACH (halfedge_descriptor hd, halfedges(m_M)) {
    double weight = m_weight_calculator(hd);
    size_t i = size_t(m_M.source(hd));
    size_t j = size_t(m_M.target(hd));
    double Lij =
        /*2**/ weight; // multiplied with 2 because m_weight_calculator's
                       // operator() returns cota/2+cotb/2
    diagonalElements[i] -= Lij;
    tripletVector.push_back(Triplet(i, j, Lij));
  }

  // populate diagonal elements
  for (size_t i = 0; i < diagonalElements.size(); i++) {
    tripletVector.push_back(Triplet(i, i, diagonalElements[i]));
  }
  m_L.setFromTriplets(tripletVector.begin(), tripletVector.end());
}

CGALSurfaceMesh MeshContractor::getContractedMesh() const {
  // if (m_M.has_garbage()) {
  //  std::cout << "Mesh has garbage.Collecting it.." << std::endl;
  //  m_M.collect_garbage();
  //}
  return m_M;
}
