#include "meshcontractor.h"
#include "debug_meshcontractor.h"
SpMatrix concatenateVertically(SpMatrix A, SpMatrix B) {
  // A
  //---
  // B
  assert(A.cols() == B.cols());
  std::vector<EigenTriplet> tripletVector;
  for (int k = 0; k < A.outerSize(); ++k) {
    for (SpMatrix::InnerIterator it(A, k); it; ++it) {
      tripletVector.push_back(EigenTriplet(it.row(), it.col(), it.value()));
    }
    for (SpMatrix::InnerIterator it(B, k); it; ++it) {
      tripletVector.push_back(
          EigenTriplet(it.row() + A.rows(), it.col(), it.value()));
    }
  }
  SpMatrix M(A.rows() + B.rows(), A.cols());
  M.setFromTriplets(tripletVector.begin(), tripletVector.end());

  return M;
}

std::vector<size_t> MeshContractor::getFixedVertices() {
  return std::vector<size_t>(fixedVertices.begin(), fixedVertices.end());
}

MeshContractor::MeshContractor(CGALSurfaceMesh meshToContract,
                               std::vector<uint> indices)
    : m_M(meshToContract) {

  Eigen::initParallel();

  Eigen::setNbThreads(6);
  // int nthreads = Eigen::nbThreads();
  // std::cout << "THREADS = " << nthreads << std::endl; // returns '1'

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
  m_originalVolume = CGAL::Polygon_mesh_processing::volume(m_M);
  m_Wh = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
  m_Wh.setIdentity();
  m_Wh *= m_Wh0;

  double averageFaceArea =
      CGAL::Polygon_mesh_processing::area(m_M) / (m_M.number_of_faces());
  m_Wl = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
  m_Wl.setIdentity();
  double WlInitialWeight = 0.001 * std::sqrt(averageFaceArea);

  m_Wl *= WlInitialWeight;
  // computeLaplaceOperatorUsingCGAL(); //?

  uint numRowsF = indices.size() / 3;
  F.resize(numRowsF, 3);
  for (int i = 0; i < indices.size(); i += 3) {
    uint faceIndex = i / 3;
    F(faceIndex, 0) = indices[i];
    F(faceIndex, 1) = indices[i + 1];
    F(faceIndex, 2) = indices[i + 2];
  }
  isVertexFixed.resize(m_M.number_of_vertices(), false);
  computeLaplaceOperator();
  m_A = Vector::Zero(m_M.number_of_vertices());
  computeOneRingAreaVector();
  m_A0 = m_A;
  m_previousDeltaCoords = EigenMatrix::Zero(m_M.number_of_vertices(), 3);
}

void MeshContractor::contractMesh(const double volumeThresholdRatio) {
  while (CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume >
             volumeThresholdRatio &&
         m_iterationsCompleted < m_maxNumOfIterations) {
    executeContractionStep();
  }
}

void MeshContractor::printFixedVertices(EigenMatrix verticesMatrix) {
  std::vector<size_t> fixedVerticesIndices = getFixedVertices();
  EigenMatrix fixedVerticesPositions(fixedVerticesIndices.size(), 3);
  for (size_t i = 0; i < fixedVerticesPositions.rows(); i++) {
    fixedVerticesPositions(i, 0) = verticesMatrix(fixedVerticesIndices[i], 0);
    fixedVerticesPositions(i, 1) = verticesMatrix(fixedVerticesIndices[i], 1);
    fixedVerticesPositions(i, 2) = verticesMatrix(fixedVerticesIndices[i], 2);
  }
  std::cout << "Fixed vertices positions are:" << std::endl;
  std::cout << fixedVerticesPositions << std::endl;
}

void MeshContractor::executeContractionStep() {
  std::cout << "Contracting Mesh.." << std::endl;
  EigenMatrix V = constructVertexMatrix();
  // printMatrix(V, "V");
  printFixedVertices(V);

  V = solveForNewVertexPositions(V);
  printFixedVertices(V);
  // printMatrix(V, "NewV");
  updateMeshPositions(V);
  // printSparseMatrix(m_Wl, "Wl");
  updateWl();
  // printVector(m_A, "m_A");
  computeOneRingAreaVector();
  // printSparseMatrix(m_Wh, "Wh");
  updateWh();
  // printDiagonalElementsOfSparseMatrix(m_L, "L");
  m_L = computeLaplaceOperatorUsingIGL();
  m_previousDeltaCoords = m_L * V;
  // computeLaplaceOperator();

  m_iterationsCompleted++;
  std::cout << "Number of iterations completed:" << m_iterationsCompleted
            << std::endl;
  std::cout << "current volume/original volume="
            << CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume
            << std::endl;
}

EigenMatrix MeshContractor::constructVertexMatrix() const {

  EigenMatrix V(m_M.number_of_vertices(), 3);
  for (auto v : m_M.vertices()) {
    const CGALSurfaceMesh::Point p = m_M.point(v);
    V(size_t(v), 0) = p.x();
    V(size_t(v), 1) = p.y();
    V(size_t(v), 2) = p.z();
  }
  return V;
}
void MeshContractor::updateMeshPositions(EigenMatrix Vnew) {
  for (auto v : m_M.vertices()) {
    m_M.point(v) = CGALSurfaceMesh::Point(
        Vnew(size_t(v), 0), Vnew(size_t(v), 1), Vnew(size_t(v), 2));
  }
}

EigenMatrix MeshContractor::solveForNewVertexPositions(
    EigenMatrix currentVertexPositions) /*const*/ {

  std::vector<size_t> fixedIndices = getFixedVertices();
  for (size_t index : fixedIndices) {
    m_Wl.coeffRef(index, index) = 1;
    // m_Wh.coeffRef(index, index) = 1;
  }

  SpMatrix WlL = m_Wl * m_L;

  SpMatrix A = concatenateVertically(WlL, m_Wh);
  // SpMatrix A(WlL.rows() + m_Wh.rows(), WlL.cols());
  // A << WlL, m_Wh;
  EigenMatrix Bupper =
      m_previousDeltaCoords; // EigenMatrix::Zero(WlL.rows(), 3);
  // for (size_t vi: fixedIndices) {
  //  Bupper(vi, 0) = m_previousDeltaCoords(vi, 0);
  //  Bupper(vi, 1) = m_previousDeltaCoords(vi, 1);
  //  Bupper(vi, 2) = m_previousDeltaCoords(vi, 2);
  //}
  EigenMatrix Blower = (m_Wh * currentVertexPositions);
  EigenMatrix B(Bupper.rows() + Blower.rows(), 3);
  B << Bupper, Blower;

  // Solve system using SimplicialLDLT
  Eigen::SimplicialLDLT<SpMatrix> solver;
  SpMatrix At = A.transpose();
  SpMatrix AtA = A.transpose() * A;
  solver.compute(AtA);
  EigenMatrix AtB = A.transpose() * B;

  EigenMatrix newVertexPositions = solver.solve(AtB);

  assert(!hasInfinity(WlL));
  assert(!hasNaN(WlL));
  assert(!hasInfinity(B));
  assert(!hasNaN(B));
  assert(!hasNaN(At));
  assert(!hasNaN(A));
  assert(!hasInfinity(At));
  assert(!hasInfinity(A));
  assert(!hasNaN(AtA));
  assert(!hasInfinity(AtA));
  assert(!hasNaN(AtB));
  assert(!hasInfinity(AtB));
  assert(!hasNaN(newVertexPositions));
  assert(!hasInfinity(newVertexPositions));

  return newVertexPositions;
}

void MeshContractor::updateWl() { m_Wl *= m_Sl; }

void MeshContractor::updateWh() {
  double threshold = 10;
  for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
    assert(m_A(i) != 0);
    double m_Whi = std::sqrt(m_A0(i) / m_A(i));
    assert(m_Whi != std::numeric_limits<double>::infinity());
    assert(!std::isnan(m_Whi));
    m_Wh.coeffRef(i, i) = m_Whi;
    if (m_Whi > threshold) {
      isVertexFixed[i] = true;
      fixedVertices.insert(i);
    }
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
  assert(v ==
         m_M.target(
             m_M.halfedge(v))); // make sure target and not source is being used
}

void MeshContractor::computeLaplaceOperator() {

  m_L = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());

  std::vector<EigenTriplet> tripletVector;
  std::vector<double> diagonalElements(m_M.number_of_vertices(), 0);
  // Weight_calculator m_weight_calculator(m_M);
  // SpMatrix igl_L = computeLaplaceOperatorUsingIGL();
  for (auto ei : m_M.edges()) {
    size_t i = size_t(m_M.vertex(ei, 0));
    size_t j = size_t(m_M.vertex(ei, 1));
    double weight = computeCotangentWeight(ei) / 2;
    double Lij = 1;
    assert(!std::isnan(diagonalElements[i] - Lij));
    diagonalElements[i] -= Lij * weight;
    diagonalElements[j] -= Lij * weight;
    tripletVector.push_back(EigenTriplet(i, j, Lij * weight));
    tripletVector.push_back(EigenTriplet(j, i, Lij * weight));
  }

  // populate diagonal elements
  for (size_t i = 0; i < diagonalElements.size(); i++) {
    tripletVector.push_back(EigenTriplet(i, i, diagonalElements[i]));
  }
  m_L.setFromTriplets(tripletVector.begin(), tripletVector.end());
  assert(!hasNaN(m_L));
}
double
MeshContractor::computeCotangentWeight(CGALSurfaceMesh::edge_index ei) const {
  CGALSurfaceMesh::halfedge_index hei = m_M.halfedge(ei, 0);
  return computeCotangentWeight(hei);
}

double MeshContractor::computeCotangentWeight(
    CGALSurfaceMesh::halfedge_index hei) const {

  CGALSurfaceMesh::Point a = m_M.point(m_M.source(hei)),
                         b = m_M.point(m_M.target(hei));
  CGALSurfaceMesh::Point c = m_M.point(m_M.target(m_M.next(hei))),
                         d = m_M.point(m_M.target(m_M.next(m_M.opposite(hei))));

  assert(c != b);
  assert(c != a);
  assert(d != a);
  assert(d != b);
  Kernel::Vector_3 v1 = Kernel::Vector_3(c, b), v2 = Kernel::Vector_3(c, a),
                   v3 = Kernel::Vector_3(d, a), v4 = Kernel::Vector_3(d, b);

  double cot1 = computeCotangentValue(v1, v2);
  double cot2 = computeCotangentValue(v3, v4);
  double cotWeight = cot1 + cot2;
  assert(!std::isnan(cotWeight) && !std::isinf(cotWeight));
  return cotWeight > 2 * maxNumber ? maxNumber : cotWeight;
}
double MeshContractor::computeCotangentValue(Kernel::Vector_3 a,
                                             Kernel::Vector_3 b) const {
  double dot_ab = a * b;
  double dot_aa = a.squared_length();
  double dot_bb = b.squared_length();

  assert(!std::isinf(dot_ab));
  assert(dot_aa != 0 && !std::isinf(dot_aa));
  assert(dot_bb != 0 && !std::isinf(dot_bb));

  double cosine = dot_ab / CGAL::sqrt(dot_aa) / CGAL::sqrt(dot_bb);
  // cosine = (cosine < lb) ? lb : cosine;
  // cosine = (cosine > ub) ? ub : cosine;
  double cosineSquared = cosine * cosine;
  double sineSquared = 1.0 - cosineSquared;
  if (sineSquared < 0)
    assert(std::abs(sineSquared) < std::pow(10, -6));
  double sine = std::sqrt(sineSquared < 0 ? 0 : sineSquared);
  assert(!std::isnan(sine));
  if (sine == 0)
    return maxNumber;
  return cosine / sine;
}

CGALSurfaceMesh MeshContractor::getContractedMesh() const { return m_M; }
