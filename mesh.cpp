#include "mesh.h"

// bool icompare_pred(unsigned char a, unsigned char b) {
//  return std::tolower(a) == std::tolower(b);
//}
//
// bool iscompare(std::string const &a, std::string const &b) {
//  if (a.length() == b.length()) {
//    return std::equal(b.begin(), b.end(), a.begin(), icompare_pred);
//  } else {
//    return false;
//  }
//}
// bool isAnObjFile(std::string filename) {
//  std::string fileExtension(filename.end() - 3, filename.end());
//  return iscompare(fileExtension, "obj");
//}
//
// bool isAnOffFile(std::string filename) {
//  std::string fileExtension(filename.end() - 3, filename.end());
//  return iscompare(fileExtension, "off");
//}

void Mesh::loadPointSphere(PointSphere ps) { m_PS = ps; }

void Mesh::handle_showVerticesStateChange(int state) {
  m_showPointSpheresOnVertices = bool(state);
}

void Mesh::handle_paintEdge() {
  // std::vector<size_t> edge{0 + edgeCounter, 1 + edgeCounter};
  skeleton.clear();
  // std::vector<size_t> edge0{0, 1}, edge1{3, 4}, edge2{5, 6};
  // std::vector<size_t> edge{0, 3};
  // std::vector<std::vector<size_t>> edges{edge0, edge1, edge2};
  std::vector<std::vector<size_t>> edges{{3, 0, 2, 1}};
  addToSkeleton(edges, m_M);
  // edgeCounter++;
  // skeleton.append({}, {m_M.point(CGALSurfaceMesh::vertex_index(0)),
  //                     m_M.point(CGALSurfaceMesh::vertex_index(1))});

  // NOTE H SEIRA PAIZEI ROLO! AYTO EINAI LATHOS
  // if (edgeCounter % 2 == 0) {
  //  skeleton.append(
  //      {}, {CGALSurfaceMesh::Point(0, 0, 0), CGALSurfaceMesh::Point(1, 1,
  //      0)});
  //  skeleton.append({{0, 1}}, {CGALSurfaceMesh::Point(0, 0, 0),
  //                             CGALSurfaceMesh::Point(10, 0, 0)});
  //} else {
  //  skeleton.append(
  //      {}, {CGALSurfaceMesh::Point(0, 1, 0), CGALSurfaceMesh::Point(0, -1,
  //      0),
  //           CGALSurfaceMesh::Point(1, 0, 0), CGALSurfaceMesh::Point(0, 1,
  //           0)});
  //}
  // skeleton.m_vertices[0].x += step;
  // skeleton.m_vertices.clear();
  // skeleton.m_indices.clear();
  // skeleton.m_vertices.push_back(glm::vec3(-1.0 + pointStep, 0, 0));
  // skeleton.m_vertices.push_back(glm::vec3(1.5, 0, 0));
  // skeleton.m_indices.push_back(0);
  // skeleton.m_indices.push_back(1);
}

void Mesh::resetMeshAttributes() {
  m_indices.clear();
  m_vertices.clear();
  centerOfMass = glm::vec3(0.0);
  m_modelMatrix = glm::mat4(1.0);
  segmentsComputed = false;
  m_perSegmentSkeletonEdges.clear();
  m_showContractedSegment = false;
  skeleton.clear();
  alphaValue = 1;
}

void Mesh::load(std::string filename) {
  resetMeshAttributes();
  populateVerticesAndIndices(filename);
  normalizeMeshViaModelMatrix();
  initializeDrawingBuffers();

  skeleton.initializeDrawingBuffers();
  segment.initialize();

  createCGALSurfaceMesh();
  // MC = MeshContractor(M, indices);
  MC = MeshContractor(m_M);
  // computeSegments();

  printMeshInformation();
}

void Mesh::assignSegmentColors() {
  for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
    std::size_t segmentIndex = segmentFaceMap[fIndex];
    glm::vec3 color = colorPalette[segmentIndex];
    for (CGALSurfaceMesh::vertex_index vi :
         m_M.vertices_around_face(m_M.halfedge(fIndex))) {
      m_vertices[size_t(vi)].Color = color;
    }
  }
}

void Mesh::colorPickedSegment() {
  // for (CGALSurfaceMesh::Face_iterator it = m_M.faces_begin();
  //     it != m_M.faces_end(); it++) {
  for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
    size_t segmentIndex = getCorrespondingSegmentIndex(fIndex);
    glm::vec3 color;
    glm::vec3 grey = glm::vec3(0.6, 0.6, 0.5);
    if (segmentIndex == selectedSegmentIndex.get()) {
      color = colorPalette[selectedSegmentIndex.get()];
    } else {
      color = grey;
    }
    for (CGALSurfaceMesh::vertex_index vi :
         m_M.vertices_around_face(m_M.halfedge(fIndex))) {
      m_vertices[size_t(vi)].Color = color;
    }
  }
}

size_t Mesh::constructSegmentMap() {
  std::cout << "Segmenting mesh.." << std::endl;
  // create a property-map ean are you sure the edge sean are you sure the
  // edge shared by two triangles is not duplicatehared by two triangles is
  // not duplicatefor segment-id
  segmentFaceMap =
      m_M.add_property_map<face_descriptor, std::size_t>("f:sid").first;
  ;
  segmentsComputed = true;
  // segment the mesh using default parameters for number of levels, and
  // smoothing lambda
  // Any other scalar values can be used instead of using SDF values computed
  // using the CGAL function
  std::size_t number_of_segments =
      CGAL::segmentation_via_sdf_values(m_M, segmentFaceMap);

  std::cout << "Number of segments: " << number_of_segments << std::endl;
  return number_of_segments;
}

void Mesh::inflationDeflationDeformer(float deformationFactor) {
  for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
    if (getCorrespondingSegmentIndex(fIndex) == selectedSegmentIndex.get()) {
      for (CGALSurfaceMesh::vertex_index vi :
           m_M.vertices_around_face(m_M.halfedge(fIndex))) {
        glm::vec3 deformationVector =
            deformationFactor * m_vertices[size_t(vi)].Normal;
        m_vertices[size_t(vi)].Position += deformationVector;
        CGALSurfaceMesh::Point previousPosition(m_M.point(vi));
        m_M.point(vi) =
            CGALSurfaceMesh::Point(previousPosition.x() + deformationVector.x,
                                   previousPosition.y() + deformationVector.y,
                                   previousPosition.z() + deformationVector.z);
      }
    }
  }
}

// void Mesh::setIntersectingTriangleUniform(int faceIndex) {
//  int location =
//      glGetUniformLocation(modelShader->programID, "intersectingFace");
//  glUniform1i(location, int(faceIndex));
//}

void Mesh::handle_drawing(Shader *shader, Shader *skeletonShader,
                          glm::mat4 projectionViewMat) {

  skeletonShader->Use();
  GLint MVPLoc = glGetUniformLocation(skeletonShader->programID, "MVP");
  if (MVPLoc == -1)
    std::cout << "Could not find MVP." << std::endl;
  else
    glUniformMatrix4fv(glGetUniformLocation(skeletonShader->programID, "MVP"),
                       1, GL_FALSE,
                       glm::value_ptr(projectionViewMat * m_modelMatrix));

  skeleton.Draw(skeletonShader, shader);

  if (m_showContractedSegment) {
    segment.handle_drawing(shader);
  }

  drawThisMesh(shader);

  if (m_showPointSpheresOnVertices) {
    // draw specific vertices
    for (PointSphere s : fixedVerticesDrawingVector) {
      s.handle_drawing(shader, m_modelMatrix);
    }
  }
}

void Mesh::drawThisMesh(Shader *shader) {
  setUniforms(shader);
  drawMesh();
}

void Mesh::setUniforms(Shader *shader) {
  shader->Use();
  // modelShader = shader; // NOTE wtf is this?
  material.setUniforms(shader);
  glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1,
                     GL_FALSE, glm::value_ptr(m_modelMatrix));
  glUniform1f(glGetUniformLocation(shader->programID, "alpha"), alphaValue);
}

void Mesh::populateVerticesAndIndices(std::__cxx11::string filename) {
  std::tie(m_indices, m_vertices) =
      meshLoader::load(filename); // vertices contains coords & normals
}

std::size_t Mesh::getCorrespondingSegmentIndex(
    const CGALSurfaceMesh::Face_index face_index) const {
  return segmentFaceMap[face_index];
}

Ray_intersection Mesh::intersects(Kernel::Ray_3 ray) const {

  Tree tree(faces(m_M).first, faces(m_M).second, m_M);

  Ray_intersection intersection = tree.first_intersection(ray);
  return intersection;
}

// int Mesh::findClosestVertex(Point intersectionPoint,
//                            CGALSurfaceMesh::Face_index intersectingFaceIndex)
//                            {
//  CGALSurfaceMesh::halfedge_index beginHalfedge =
//      m_M.halfedge(intersectingFaceIndex);
//  CGALSurfaceMesh::halfedge_index h = beginHalfedge;
//  CGALSurfaceMesh::vertex_index vIndex = m_M.source(h);
//  Kernel::Point_3 vertex = m_M.point(vIndex);
//  double minDistance = CGAL::squared_distance(vertex, intersectionPoint);
//  int closestVertexIndex = (int)vIndex;
//  do {
//    std::cout << h << std::endl;
//    vIndex = m_M.source(h);
//    Kernel::Point_3 vertex = m_M.point(vIndex);
//    double distance = CGAL::squared_distance(vertex, intersectionPoint);
//    if (distance < minDistance) {
//
//      minDistance = distance;
//      vIndex = m_M.source(h);
//      closestVertexIndex = int(vIndex);
//    }
//    h = m_M.next(h);
//  } while (h != beginHalfedge);
//  return closestVertexIndex;
//}

void Mesh::normalizeMeshViaModelMatrix() {
  centerOfMass = meshMeasuring::findCenterOfMass(m_vertices);
  maxDim = meshMeasuring::findMaxDimension(m_vertices);
  m_modelMatrix = glm::mat4(1.0);
  float scaleFactor = 1.0 / maxDim;
  m_modelMatrix = glm::scale(m_modelMatrix, glm::vec3(scaleFactor));

  glm::vec3 translationVector(-centerOfMass);
  m_modelMatrix = glm::translate(m_modelMatrix, translationVector);
}

MeshSegment Mesh::getMeshSegment() const {
  assert(selectedSegmentIndex);
  std::cout << "Using segment " << selectedSegmentIndex.get()
            << " to construct MeshSegment." << std::endl;
  MeshSegment S(m_M, m_vertices, segmentFaceMap, selectedSegmentIndex.get(),
                m_modelMatrix);
  return S;
}
MeshSegment Mesh::getMeshSegment(size_t segmentIndex) const {
  MeshSegment S(m_M, m_vertices, segmentFaceMap, segmentIndex, m_modelMatrix);
  return S;
}

void Mesh::handle_segmentSelection(Ray_intersection intersection) {
  CGALSurfaceMesh::Face_index intersectingFaceIndex = intersection->second;
  selectedSegmentIndex = getCorrespondingSegmentIndex(intersectingFaceIndex);

  std::cout << "Intersection segment:" << selectedSegmentIndex.get()
            << std::endl;
  colorPickedSegment();
  updateMeshBuffers();

  segment.setSegment(getMeshSegment());
  SMC = MeshContractor(segment.M());
}

size_t Mesh::computeSegments() {
  segmentsComputed = true;
  return constructSegmentMap();
}

void Mesh::handle_showSegments() {
  if (!segmentsComputed) {
    size_t numberOfSegments = computeSegments();
    assignSegmentColors();
    m_perSegmentSkeletonEdges.resize(numberOfSegments);
  }
  alphaValue = 1;
  assignSegmentColors();
  updateMeshBuffers();
  m_showContractedSegment = false;
}

void Mesh::inflation_handler() { // TODO δεν βρισκει τομη ακτινας με το mesh
  // μετα το deformation
  if (selectedSegmentIndex) {
    float deformationFactor = 0.01;
    inflationDeflationDeformer(deformationFactor);
    updateMeshBuffers();
  }
}

void Mesh::deflation_handler() {
  if (selectedSegmentIndex) {
    float deformationFactor = -0.01;
    inflationDeflationDeformer(deformationFactor);
    updateMeshBuffers();
  }
}

void Mesh::unselectSegment() { selectedSegmentIndex = boost::none; }

void Mesh::handle_saveModel(std::__cxx11::string destinationPathAndFileName) {
  std::ofstream outFile;
  outFile.open(destinationPathAndFileName, std::ios::out);
  if (!outFile) {
    std::cerr << "Can't save file: " << destinationPathAndFileName << std::endl;
  } else {
    CGAL::write_off(outFile, m_M);
  }
  outFile.close();
}

void Mesh::handle_saveSegment(std::__cxx11::string destinationPathAndFileName) {
  MeshSegment segment = getMeshSegment();
  std::ofstream outFile;
  outFile.open(destinationPathAndFileName, std::ios::out);
  if (!outFile) {
    std::cerr << "Can't save file: " << destinationPathAndFileName << std::endl;
  } else {
    CGAL::write_off(outFile, segment.M());
  }
  outFile.close();
}
// std::vector<size_t> Mesh::getVertexIndicesWithHighLaplacianValue() {
//  return MC.getVertexIndicesWithHighLaplacianValue();
//}

void Mesh::handle_meshContractionReversing() {
  MC.executeContractionReversingStep();
  m_M = MC.getContractedMesh();
  DrawableMesh::updateDrawingVertices();
  updateMeshBuffers();

  fixedVerticesDrawingVector.clear();
  for (size_t vi : MC.getFixedVertices()) {
    PointSphere ps = m_PS;
    ps.setPosition(m_M.point(CGALSurfaceMesh::vertex_index(vi)));
    ps.setColor(glm::vec3(1, 0, 0));
    fixedVerticesDrawingVector.push_back(ps);
  }
}

void Mesh::handle_meshContraction() {
  if (m_M.has_garbage())
    m_M.collect_garbage();

  MC.executeContractionStep();
  m_M = MC.getContractedMesh();

  DrawableMesh::updateDrawingVertices();
  updateMeshBuffers();

  fixedVerticesDrawingVector.clear();
  for (size_t vi : MC.getFixedVertices()) {
    PointSphere ps = m_PS;
    ps.setPosition(m_M.point(CGALSurfaceMesh::vertex_index(vi)));
    ps.setColor(glm::vec3(1, 0, 0));
    fixedVerticesDrawingVector.push_back(ps);
  }
}

void Mesh::handle_segmentContraction() {
  alphaValue = 0.4;
  // SMC.contractMesh(std::pow(10, -4));
  SMC.executeContractionStep();
  CGALSurfaceMesh contractedSegment = SMC.getContractedMesh();
  segment.setM(contractedSegment);
  m_showContractedSegment = true;
}

void Mesh::addToSkeleton(
    std::vector<std::vector<size_t>> skeletonEdgesInMeshIndices,
    const CGALSurfaceMesh &M) {
  using MeshIndex = size_t;
  using SkeletonIndex = size_t;
  std::unordered_set<size_t> skeletonNodesInMeshIndices;

  for (auto skeletonEdge : skeletonEdgesInMeshIndices) {
    for (auto v : skeletonEdge) {
      skeletonNodesInMeshIndices.insert(v);
    }
  }
  std::vector<CGALSurfaceMesh::Point> skeletonNodePositions;
  std::map<MeshIndex, SkeletonIndex> meshIndices_to_skeletonIndices;
  SkeletonIndex indexInSkeleton = 0;
  for (MeshIndex indexInMesh : skeletonNodesInMeshIndices) {
    // skeletonNodePositions.push_back(
    //    CGALSurfaceMesh::Point(1 + indexInMesh, 2 + indexInMesh, 0));
    skeletonNodePositions.push_back(
        M.point(CGALSurfaceMesh::vertex_index(indexInMesh)));
    meshIndices_to_skeletonIndices.insert(
        std::make_pair(indexInMesh, indexInSkeleton));
    indexInSkeleton++;
  }

  // skeletonEdges with mesh indices -> skeletonEdges with skeleton indices

  std::vector<std::vector<size_t>> skeletonEdgesInSkeletonIndices;
  for (auto edge : skeletonEdgesInMeshIndices) {
    skeletonEdgesInSkeletonIndices.push_back(
        std::vector<size_t>{meshIndices_to_skeletonIndices[edge[0]],
                            meshIndices_to_skeletonIndices[edge[1]]});
  }

  skeleton.append(skeletonEdgesInSkeletonIndices, skeletonNodePositions);
}
void Mesh::handle_meshConnectivitySurgery() {
  ConnectivitySurgeon CS(m_M);
  CS.extract_skeleton();
  auto skeletonEdgesInMeshIndices = CS.getSkeleton();
  m_skeletonMeshMapping = CS.getSkeletonMeshMapping();

  addToSkeleton(skeletonEdgesInMeshIndices, m_M);
}

void Mesh::handle_meshRefinementEmbedding() {
  // m_skeletonNodes.clear(); // forget skeleton node drawing vector.

  // for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
  // std::vector<size_t> localVerticesThatMapToThisNode =
  // m_skeletonMeshMapping[i];
  // if (localVerticesThatMapToThisNode.empty())
  //    continue;
  // CGALSurfaceMesh::Point centerOfMass =
  // computeCenterOfMass(localVerticesThatMapToThisNode);

  // PointSphere tempPS = m_PS;
  // tempPS.setPosition(centerOfMass);
  // tempPS.setColor(glm::vec3(1, 0, 0));
  // tempPS.doubleSize();
  // m_skeletonNodes.push_back(tempPS);
  //}
}

CGALSurfaceMesh::Point
Mesh::computeCenterOfMass(std::vector<size_t> vertexIndices) {
  Kernel::Vector_3 sum(CGAL::NULL_VECTOR);
  for (size_t vIndex : vertexIndices) {
    CGALSurfaceMesh::Point p = m_M.point(CGALSurfaceMesh::vertex_index(vIndex));
    sum += Kernel::Vector_3(p.x(), p.y(), p.z());
  }
  Kernel::Vector_3 centerOfMassVector = sum / vertexIndices.size();
  CGALSurfaceMesh::Point centerOfMass(
      centerOfMassVector.x(), centerOfMassVector.y(), centerOfMassVector.z());
  return centerOfMass;
}

void Mesh::handle_segmentConnectivitySurgery() {
  assert(selectedSegmentIndex);
  CGALSurfaceMesh contractedSegment = segment.M();
  ConnectivitySurgeon CS(contractedSegment);
  CS.extract_skeleton();
  auto skeletonEdgesInMeshIndices = CS.getSkeleton();

  addToSkeleton(skeletonEdgesInMeshIndices, contractedSegment);
  m_showContractedSegment = false;
  // std::set<size_t> skeletonNodes;
  // for (auto skeletonEdge : skeletonEdges) {
  //    for (auto v : skeletonEdge) {
  //        skeletonNodes.insert(
  //    	v); // v is the index in the segment and not in the original
  //    mesh
  //    }
  //}

  // for (auto vIndex : skeletonNodes) {
  //    PointSphere tempPS = m_PS;
  //    auto p(contractedSegment.point(CGALSurfaceMesh::vertex_index(vIndex)));
  //    tempPS.setPosition(p);
  //    tempPS.setColor(glm::vec3(1, 0, 0));
  //    tempPS.doubleSize();
  //    m_skeletonNodes.push_back(tempPS);
  //}
  // m_showContractedSegment = false;
}

void Mesh::updateVertices(const MeshSegment &copyFrom) {
  updateDrawingVertices(copyFrom);
  updateCGALSurfaceMeshVertices(copyFrom);
}

void Mesh::updateDrawingVertices(const MeshSegment &copyFrom) {
  size_t index = 0;
  for (auto v : copyFrom.M().vertices()) { // update drawing vertices in model
    CGALSurfaceMesh::Point p = copyFrom.M().point(v);
    m_vertices[copyFrom.vertexCorrespondence[index]].Position =
        glm::vec3(p.x(), p.y(), p.z());
    index++;
  }
}
void Mesh::updateCGALSurfaceMeshVertices(const MeshSegment &copyFrom) {
  size_t index = 0;
  for (auto v : copyFrom.M().vertices()) {
    CGALSurfaceMesh::Point p = copyFrom.M().point(v);
    CGALSurfaceMesh::vertex_index vInOriginalMesh(
        copyFrom.vertexCorrespondence[index]);
    m_M.point(vInOriginalMesh) = CGALSurfaceMesh::Point(p.x(), p.y(), p.z());
    index++;
  }
}
