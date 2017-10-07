//#include "connectivitysurgeon.h"
//
// ConnectivitySurgeon::ConnectivitySurgeon(CGALSurfaceMesh &contractedMesh)
//    : m_M(contractedMesh), m_edgeRemover(contractedMesh) {
//  initializeConnectivitySurgery();
//  executeConnectivitySurgery();
//}
//
// void ConnectivitySurgeon::initializeConnectivitySurgery() {
//  if (m_M.has_garbage()) {
//    std::cout << "Mesh has garbage.Collecting it.." << std::endl;
//    m_M.collect_garbage();
//  }
//  initializeQMatrix();
//  initializeShapeCostMatrix();
//  initializeSamplingCostMatrix();
//  initializeTotalCost();
//}
//
// void ConnectivitySurgeon::initializeQMatrix() {
//
//  for (size_t vertexIndex = 0; vertexIndex < m_M.number_of_vertices();
//       vertexIndex++) {
//    // get cgal's vertex_index
//    CGALSurfaceMesh::vertex_index vi(vertexIndex);
//    // get all halfedges that have as source vi
//    auto hei_range = CGAL::halfedges_around_source(vi, m_M);
//
//    Eigen::Matrix4d Qi = Eigen::Matrix4d::Zero();
//    // iterate over all of these half edges
//    for (CGALSurfaceMesh::halfedge_index hei : hei_range) {
//      CGALSurfaceMesh::Point Vi = m_M.point(m_M.source(hei)),
//                             Vj = m_M.point(m_M.target(hei));
//      Matrix34d Kij = computeKMatrix(Vi, Vj);
//      Qi += Kij.transpose() * Kij;
//    }
//    Q.push_back(Qi);
//  }
//}
//
// void ConnectivitySurgeon::initializeShapeCostMatrix() {
//  for (CGALSurfaceMesh::halfedge_index hei : m_M.halfedges()) {
//    CGALSurfaceMesh::vertex_index Vi = m_M.source(hei), Vj = m_M.target(hei);
//    size_t i = Vi;
//    size_t j = Vj;
//    double Fi = computeF(i, m_M.point(Vj));
//    double Fj = computeF(j, m_M.point(Vj));
//    double F = Fi + Fj;
//    shapeCostMatrix.push_back(F);
//  }
//}
//
// Matrix34d ConnectivitySurgeon::computeKMatrix(CGALSurfaceMesh::Point Vi,
//                                              CGALSurfaceMesh::Point Vj) const
//                                              {
//  Kernel::Vector_3 edge_vector(Vj.x() - Vi.x(), Vj.y() - Vi.y(),
//                               Vj.z() - Vi.z());
//  Kernel::Vector_3 a = edge_vector / CGAL::sqrt(edge_vector.squared_length());
//
//  Kernel::Vector_3 b =
//      CGAL::cross_product(a, Kernel::Vector_3(Vi.x(), Vi.y(), Vi.z()));
//  Matrix34d K;
//  K << 0, -a.z(), a.y(), -b.x(), a.z(), 0, -a.x(), -b.y(), -a.y(), a.x(), 0,
//      -b.z();
//  return K;
//}
//
// double ConnectivitySurgeon::computeF(size_t vertexIndex,
//                                     CGALSurfaceMesh::Point p) const {
//  Eigen::Vector4d P(p.x(), p.y(), p.z(), 1);
//  return P.transpose() * Q[vertexIndex].get() * P;
//}
//
// void ConnectivitySurgeon::initializeSamplingCostMatrix() // NOTE iterate over
//                                                         // edges
//// instead of halfedges?
//{
//  for (CGALSurfaceMesh::halfedge_index hei : m_M.halfedges()) {
//    double F = computeSamplingCost(hei);
//    samplingCostMatrix.push_back(F);
//  }
//}
//
// double ConnectivitySurgeon::computeSamplingCost(he_index h) const {
//  v_index i = m_M.source(h), j = m_M.target(h);
//  CGALSurfaceMesh::Point Pi = m_M.point(i), Pj = m_M.point(j);
//  double ijDistance = CGAL::sqrt(
//      (Kernel::Vector_3(Pi, Pj)).squared_length()); // CGALSurfaceMesh =
//  // CGAL::Surface_mesh<Kernel>.  NOTE
//  // if i iterated over all edges
//  // instead of halfedges i would do
//  // this operation the half times.
//
//  double sumIKDistances =
//      0; // sum of distances of the vertex i to all adjacent vertices of i
//  for (CGALSurfaceMesh::halfedge_index hi :
//       CGAL::halfedges_around_source(i, m_M)) {
//    CGALSurfaceMesh::vertex_index adjacentVertexIndex = m_M.target(hi);
//    CGALSurfaceMesh::Point adjacentVertexPosition =
//        m_M.point(adjacentVertexIndex);
//    Kernel::Vector_3 edge_vector(Pi, adjacentVertexPosition);
//    sumIKDistances += CGAL::sqrt(edge_vector.squared_length());
//  }
//  double F = ijDistance * sumIKDistances;
//  return F;
//}
//
// void ConnectivitySurgeon::initializeTotalCost() {
//  for (size_t heIndex = 0; heIndex < m_M.number_of_halfedges(); heIndex++) {
//    double Ftotal =
//        samplingCostMatrix[heIndex].get() + shapeCostMatrix[heIndex].get();
//    totalCost.insert(std::make_pair(he_index(heIndex), Ftotal));
//  }
//}
//
// void ConnectivitySurgeon::executeConnectivitySurgery() {
//  while (m_M.number_of_faces() != 0) {
//    boost::optional<he_index> nextHalfedge = getNextEdgeToRemove();
//    if (!nextHalfedge)
//      break;
//    he_index heToCollapse = nextHalfedge.get();
//    markEdgeAsProcessed(heToCollapse);
//    if (edgeCollapseViolation(heToCollapse) ||
//        !CGAL::Euler::does_satisfy_link_condition(m_M.edge(heToCollapse),
//        m_M))
//      continue;
//
//    CGALSurfaceMesh::vertex_index vKept = m_M.target(heToCollapse),
//                                  vGone = m_M.source(heToCollapse);
//    std::vector<v_index> oneRingNeighboursOfvGone =
//    getOneRingNeighbours(vGone);
//
//    std::vector<he_index> additionalHEthatWereRemoved =
//        m_edgeRemover.removeEdge(heToCollapse);
//    for (auto he : additionalHEthatWereRemoved)
//      markHalfedgeAsProcessed(he);
//
//    std::vector<Pair> affectedHalfedges =
//        recalculateCostOfAffectedEdges(oneRingNeighboursOfvGone, vKept,
//        vGone);
//    updateTotalCosts(affectedHalfedges);
//    std::cout << "Number of faces left:" << m_M.number_of_faces() <<
//    std::endl;
//  }
//  std::cout << "Connectivity surgery ended." << std::endl;
//  std::cout << "Resulting Mesh has:" << m_M.number_of_vertices()
//            << " vertices and " << m_M.number_of_faces() << " faces."
//            << std::endl;
//}
//
// boost::optional<he_index> ConnectivitySurgeon::getNextEdgeToRemove() const {
//  if (totalCost.size() == 0)
//    return boost::none;
//  auto x = std::min_element(totalCost.begin(), totalCost.end(),
//                            [](const std::pair<he_index, double> &p1,
//                               const std::pair<he_index, double> &p2) {
//                              return p1.second < p2.second;
//                            });
//  return x->first;
//}
//
// void ConnectivitySurgeon::markEdgeAsProcessed(he_index he) {
//  markHalfedgeAsProcessed(he);
//  markHalfedgeAsProcessed(m_M.opposite(he));
//}
//
// void ConnectivitySurgeon::markHalfedgeAsProcessed(he_index he) {
//  totalCost.erase(he);
//  shapeCostMatrix[he] = boost::none;
//  samplingCostMatrix[he] = boost::none;
//}
//
// std::vector<v_index>
// ConnectivitySurgeon::getOneRingNeighbours(v_index v) const {
//  auto oneRingRange = CGAL::vertices_around_target(v, m_M);
//  std::vector<v_index> oneRingNeighbours;
//  for (auto vi : oneRingRange)
//    oneRingNeighbours.push_back(vi);
//
//  return oneRingNeighbours;
//}
//
// bool ConnectivitySurgeon::edgeCollapseViolation(
//    he_index he) const // needed to satisfy CGAL_preconditions in
//    collapse_edge
//{
//  he_index ohe = m_M.opposite(he);
//  he_index ophe = m_M.opposite(m_M.prev(he));
//  he_index opohe = m_M.opposite(m_M.prev(ohe));
//
//  bool topFaceExists = !CGAL::is_border(he, m_M);
//  bool bottomFaceExists = !CGAL::is_border(m_M.opposite(he), m_M);
//  if ((topFaceExists && m_M.degree(m_M.target(ophe)) <= 2) ||
//      (bottomFaceExists && m_M.degree(m_M.target(opohe)) <= 2))
//    return true;
//  else
//    return false;
//}
//
// std::vector<Pair> ConnectivitySurgeon::recalculateCostOfAffectedEdges(
//    std::vector<v_index> oneRingNeighboursOfvGone, v_index vKept,
//    v_index vGone) {
//  updateQ(vKept, vGone);
//  std::map<he_index, double> newShapeCostValues =
//      computeOneRingShapeCosts(vKept);
//  std::map<he_index, double> newSamplingCostValues =
//      computeSamplingCostsOfVerticesAsSource(oneRingNeighboursOfvGone);
//  std::map<he_index, double> totalCosts =
//      mergeMapsOfCosts(newShapeCostValues, newSamplingCostValues);
//  std::vector<Pair> totalCostsToUpdate;
//  totalCostsToUpdate.reserve(totalCosts.size());
//  std::transform(totalCosts.begin(), totalCosts.end(),
//                 std::back_inserter(totalCostsToUpdate),
//                 [](Pair const &p) { return p; });
//  return totalCostsToUpdate;
//}
//
// void ConnectivitySurgeon::updateTotalCosts(std::vector<Pair> newTotalCosts) {
//  for (const auto p : newTotalCosts) {
//    totalCost[p.first] = p.second;
//  }
//}
//
// void ConnectivitySurgeon::updateQ(v_index vKept, v_index vGone) {
//  Q[vKept] = Q[vGone].get() + Q[vKept].get();
//  Q[vGone] = boost::none;
//}
//
// std::map<he_index, double> ConnectivitySurgeon::computeOneRingShapeCosts(
//    v_index vKept) { // NOTE could I break this function into simpler ones?
//
//  std::map<he_index, double> oneRingValues;
//
//  // halfedges: (vKept->neighbour)
//  auto sourceIndex = vKept;
//  auto h_range = CGAL::halfedges_around_source(vKept, m_M);
//  auto endHei = *(h_range.end());
//  for (auto it = h_range.begin(); it != h_range.end(); it++) {
//    auto hei = *it;
//    auto targetIndex = m_M.target(hei);
//
//    CGALSurfaceMesh::Point targetPosition = m_M.point(targetIndex);
//    double Fj = computeF(targetIndex, targetPosition);
//    double Fi = computeF(sourceIndex, targetPosition);
//    double F = Fi + Fj;
//    shapeCostMatrix[hei] = F;
//    oneRingValues.insert(std::make_pair(hei, F));
//  }
//
//  // halfedges: (neighbour->vKept)
//  auto targetIndex = vKept;
//  CGALSurfaceMesh::Point targetPosition = m_M.point(targetIndex);
//  double Fj = computeF(targetIndex, targetPosition);
//  for (CGALSurfaceMesh::halfedge_index hei : CGAL::halfedges_around_target(
//           CGALSurfaceMesh::vertex_index(targetIndex), m_M)) {
//    auto sourceIndex = m_M.target(hei);
//
//    double Fj = computeF(targetIndex, targetPosition);
//    double Fi = computeF(sourceIndex, targetPosition);
//    double F = Fi + Fj;
//    shapeCostMatrix[hei] = F;
//    oneRingValues.insert(std::make_pair(hei, F));
//  }
//  return oneRingValues;
//}
//
// std::map<he_index, double>
// ConnectivitySurgeon::computeSamplingCostsOfVerticesAsSource(
//    std::vector<v_index> verticesAsSource) {
//  std::map<he_index, double> samplingCosts;
//
//  for (auto vi : verticesAsSource) {
//    for (auto hei : CGAL::halfedges_around_source(vi, m_M)) {
//      double F = computeSamplingCost(hei);
//      samplingCosts.insert(std::make_pair(hei, F));
//      samplingCostMatrix[hei] = F;
//    }
//  }
//  return samplingCosts;
//}
//
// std::map<he_index, double> ConnectivitySurgeon::mergeMapsOfCosts(
//    std::map<he_index, double> shapeCosts,
//    std::map<he_index, double> samplingCosts) const {
//  std::map<he_index, double> mergedMap(shapeCosts);
//  for (auto pair : samplingCosts) {
//    if (mergedMap.find(pair.first) ==
//        mergedMap.end()) // key does not exist in mergedMapp
//    {
//      mergedMap.insert(std::make_pair(
//          pair.first, shapeCostMatrix[pair.first].get() + pair.second));
//    } else {
//      mergedMap[pair.first] += pair.second;
//    }
//  }
//  return mergedMap;
//}
