#include "segmentcontractor.h"

void SegmentContractor::calculateSkeleton(){
    while(CGAL::Polygon_mesh_processing::volume(M)/originalVolume>volumeThreshold){
     contractMesh();
 }
}

void SegmentContractor::contractMesh(){

}
 /*void SegmentContractor::computeLaplaceOperator(int segmentIndex) {

  Eigen::MatrixXd laplaceOperator(m_P.M.number_of_vertices(),
                                m_P.M.number_of_vertices());

 CGALSurfaceMesh::Edge_range er = m_P.M.edges();

 for (auto eit = er.begin(); eit != er.end(); eit++) {
  CGALSurfaceMesh::Vertex_index edgeVertex0 = m_P.M.vertex(*eit, 0);
  CGALSurfaceMesh::Point P0 = m_P.M.point(edgeVertex0);

  CGALSurfaceMesh::Vertex_index edgeVertex1 = m_P.M.vertex(*eit, 1);
  CGALSurfaceMesh::Point P1 = m_P.M.point(edgeVertex1);

  // compute aij
  CGALSurfaceMesh::Halfedge_index halfedge0 = m_P.M.halfedge(*eit, 0);
  CGALSurfaceMesh::Halfedge_index nextHalfedge0 = m_P.M.next(halfedge0);
  CGALSurfaceMesh::Vertex_index firstVertexOppositeToEdge =
      m_P.M.target(nextHalfedge0);
  CGALSurfaceMesh::Point Pa = m_P.M.point(firstVertexOppositeToEdge);
  CGAL::Vector_3<Kernel> v1 = P0 - Pa;
  CGAL::Vector_3<Kernel> v2 = P1 - Pa;
  double cosine = v1 * v2 / CGAL::sqrt(v1 * v1) / CGAL::sqrt(v2 * v2);
  double a = std::acos(cosine); // IN RADIANS

  // compute bij
  CGALSurfaceMesh::Halfedge_index halfedge1 = m_P.M.halfedge(*eit, 1);
  CGALSurfaceMesh::Halfedge_index nextHalfedge1 = m_P.M.next(halfedge1);
  CGALSurfaceMesh::Vertex_index secondVertexOppositeToEdge =
      m_P.M.target(nextHalfedge1);
  CGALSurfaceMesh::Point Pb = m_P.M.point(secondVertexOppositeToEdge);
  v1 = P0 - Pb;
  v2 = P1 - Pb;
  cosine = v1 * v2 / CGAL::sqrt(v1 * v1) / CGAL::sqrt(v2 * v2);
  double b = std::acos(cosine); // IN RADIANS

  size_t i = size_t(edgeVertex0);
  size_t j = size_t(edgeVertex1);
  double Wij = std::cos(a) / std::sin(a) + std::cos(b) / std::sin(b);
  laplaceOperator(i, j) = Wij;
}

 for (int row = 0; row < m_P.M.number_of_vertices(); row++) {
  double rowSum = 0;
  for (int col = 0; col < m_P.M.number_of_vertices(); col++) {
    rowSum -= laplaceOperator(row, col);
  }
  laplaceOperator(row, row) = rowSum;
}
 L = laplaceOperator;
}

 void SegmentContractor::updateSurfaceMeshPositions() {
  CGALSurfaceMesh::Vertex_range vr = m_P.M.vertices();
  for (auto vit = vr.begin(); vit != vr.end(); vit++) {
    size_t vertexIndex = size_t(*vit);
    CGALSurfaceMesh::Point newPosition(currentVertices(vertexIndex, 0),
                                       currentVertices(vertexIndex, 1),
                                       currentVertices(vertexIndex, 2));
    std::cout << "Before:" << m_P.M.point(*vit) << std::endl;
    m_P.M.point(*vit) = newPosition;
    std::cout << "After:" << m_P.M.point(*vit) << std::endl;
    //    vertices[vertexIndex].Position =
    //        glm::vec3(newPosition.x(), newPosition.y(), newPosition.z());
  }
}

 Matrix SegmentContractor::getPositionsFromSurfaceMesh() {
  size_t numberOfVertices = m_P.M.number_of_vertices();
  Matrix V(numberOfVertices, 3);

  CGALSurfaceMesh::Vertex_range vr = m_P.M.vertices();
  for (auto vit = vr.begin(); vit != vr.end(); vit++) {
    auto p = m_P.M.point(*vit);
    size_t vertexIndex = size_t(*vit);
    V(vertexIndex, 0) = p.x();
    V(vertexIndex, 1) = p.y();
    V(vertexIndex, 2) = p.z();
  }
  return V;
}

 void SegmentContractor::solveForNewVertexPositions() {
  Matrix WlL = Wl * L;
  Matrix A(WlL.rows() + Wh.rows(), WlL.cols());
  A << WlL, Wh;
  Matrix Bupper(WlL.rows(), 3);
  Matrix Blower = Wh * currentVertices;
  Matrix B(Bupper.rows() + Blower.rows(), 3);
  B << Bupper, Blower;
  currentVertices =
      A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

  updatem_P.MPositions();
}
 void SegmentContractor::contract(int segmentIndex) {
  std::cout << "Contracting segment:" << segmentIndex << std::endl;

  computeLaplaceOperator(segmentIndex);
  // Wh = Matrix::Identity(numberOfVertices, numberOfVertices);
  // Wl = Matrix::Identity(numberOfVertices, numberOfVertices);
  // double averageFaceArea = computeAverageFaceArea();
  // Wl *= 0.001 * std::sqrt(averageFaceArea);

  // solveForNewVertexPositions();
  // updateMeshBuffers();
  // prepareNextContraction();
}

 double SegmentContractor::computeAverageFaceArea() {
  CGALSurfaceMesh::Face_range frange = m_P.M.faces();
  double sumArea = 0;
  for (auto fit = frange.begin(); fit != frange.end(); fit++) {
    sumArea += CGAL::Polygon_mesh_processing::face_area(*fit, m_P.M);
  }
  return sumArea / m_P.M.number_of_faces();
}

 std::vector<double> SegmentContractor::computeOneRingAreas() {
  std::vector<double> oneRingAreas;
  CGALSurfaceMesh::Vertex_range vr = m_P.M.vertices();
  for (auto vit = vr.begin(); vit != vr.end(); vit++) {
    CGALSurfaceMesh::Face_around_target_range oneRingRange =
        m_P.M.faces_around_target(Skeleton.halfedge(*vit));
    double oneRingArea = 0;
    for (auto fit = oneRingRange.begin(); fit != oneRingRange.end(); fit++) {
      oneRingArea += CGAL::Polygon_mesh_processing::face_area(*fit, m_P.M);
    }
    oneRingAreas.push_back(oneRingArea);
    // auto oneRingArea =
    // CGAL::Polygon_mesh_processing::area(oneRingRange, m_P.M);
    // std::cout << "Area around vertex " << *vit << " is:" << oneRingArea
    //           << std::endl;
    // std::cout << "Area around vertex " << *vit << " is:"
    //           << CGAL::Polygon_mesh_processing::area(oneRingRange, m_P.M)
    //           << std::endl;
  }
  return oneRingAreas;
}

 void SegmentContractor::updateWh() {
  currentOneRingAreas = computeOneRingAreas();
  for (int i = 0; i < m_P.M.number_of_vertices(); i++) {
    Wh(i, i) = std::sqrt(initialOneRingAreas[i] / currentOneRingAreas[i]);
  }
}

 void SegmentContractor::updateWl() { Wl *= Sl; }

 void SegmentContractor::updateWeightingMatrices() {
  updateWh();
  updateWl();
}

 void SegmentContractor::prepareNextContraction() {
  updateWeightingMatrices();
  computeLaplaceOperator();
}

*/
