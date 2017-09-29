#include "mesh.h"

bool icompare_pred(unsigned char a, unsigned char b) {
  return std::tolower(a) == std::tolower(b);
}

bool iscompare(std::string const &a, std::string const &b) {
  if (a.length() == b.length()) {
    return std::equal(b.begin(), b.end(), a.begin(), icompare_pred);
  } else {
    return false;
  }
}
bool isAnObjFile(std::string filename) {
  std::string fileExtension(filename.end() - 3, filename.end());
  return iscompare(fileExtension, "obj");
}

bool isAnOffFile(std::string filename) {
  std::string fileExtension(filename.end() - 3, filename.end());
  return iscompare(fileExtension, "off");
}
Mesh::Mesh() {
  for (auto &color : colorPalette) {
    color /= 255.0;
  }
}

void Mesh::resetMeshAttributes() {
  indices.clear();
  vertices.clear();
  centerOfMass = glm::vec3(0.0);
  modelMatrix = glm::mat4(1.0);
  segmentsComputed = false;
}

void Mesh::load(std::string filename) {
  if (isAnObjFile(filename)) {
    resetMeshAttributes();
    // loadObjFile(filename);
    std::tie(indices, vertices) =
        meshLoader::load(filename); // vertices contains coords & normals

    setupDrawingBuffers();
    // Find the model matrix that normalizes the mesh
    centerOfMass = meshMeasuring::findCenterOfMass(vertices);
    maxDim = meshMeasuring::findMaxDimension(vertices);
    normalizeMeshViaModelMatrix();
    buildSurfaceMesh();
    // averageEdgeLength = meshMeasuring::findAverageEdgeLength(M);
    MC = MeshContractor(M, indices);
    // MC = MeshContractor(M, indices);
  } else if (isAnOffFile(filename)) {
    // loadOffFile(filename);
    std::ifstream inputFile(filename);
    assert(inputFile);
    M.clear();
    CGAL::read_off(inputFile, M);

    // populateVertices();
    // //CGAL::Polygon_mesh_processing::compute_vertex_normals
    // populateIndices();
    centerOfMass = meshMeasuring::findCenterOfMass(vertices);
    maxDim = meshMeasuring::findMaxDimension(vertices);
    normalizeMeshViaModelMatrix();
    MC = MeshContractor(M, indices);

  } else {
    std::cerr << "The file type you selected cannot be loaded."
                 "Use an .obj or an .off file."
              << std::endl;
  }

  printMeshInformation();
}

void Mesh::assignSegmentColors() {
  for (CGALSurfaceMesh::Face_iterator it = M.faces_begin(); it != M.faces_end();
       it++) {
    CGALSurfaceMesh::Face_index fIndex(*it);
    auto r = M.vertices_around_face(M.halfedge(fIndex));
    auto vbegin = r.begin();
    std::size_t segmentID = segment_property_map[fIndex];
    glm::vec3 color = colorPalette[segmentID];
    for (auto vbegin = r.begin(); vbegin != r.end(); vbegin++) {
      std::size_t vIndex = std::size_t(*vbegin);
      vertices[vIndex].Color = color;
    }
  }
}

void Mesh::colorPickedSegment() {
  for (CGALSurfaceMesh::Face_iterator it = M.faces_begin(); it != M.faces_end();
       it++) {
    CGALSurfaceMesh::Face_index fIndex(*it);
    std::size_t segmentID = segment_property_map[fIndex];
    auto r = M.vertices_around_face(M.halfedge(fIndex));
    glm::vec3 color;
    if (segmentID == selectedSegmentIndex) {
      color = colorPalette[selectedSegmentIndex];
    } else {
      color = glm::vec3(0.6, 0.6, 0.5);
    }
    for (auto vbegin = r.begin(); vbegin != r.end(); vbegin++) {
      std::size_t vIndex = std::size_t(*vbegin);
      vertices[vIndex].Color = color;
    }
  }
}

void Mesh::constructSegmentMap() {
  std::cout << "Segmenting mesh.." << std::endl;
  // create a property-map ean are you sure the edge sean are you sure the
  // edge shared by two triangles is not duplicatehared by two triangles is
  // not duplicatefor segment-id
  segment_property_map =
      M.add_property_map<face_descriptor, std::size_t>("f:sid").first;
  ;
  segmentsComputed = true;
  // segment the mesh using default parameters for number of levels, and
  // smoothing lambda
  // Any other scalar values can be used instead of using SDF values computed
  // using the CGAL function
  std::size_t number_of_segments =
      CGAL::segmentation_via_sdf_values(M, segment_property_map);

  std::cout << "Number of segments: " << number_of_segments << std::endl;
}

bool Mesh::segmentHasBeenSelected() const { return selectedSegmentIndex != -1; }

void Mesh::inflationDeflationDeformer(float deformationFactor) {
  for (CGALSurfaceMesh::Face_iterator it = M.faces_begin(); it != M.faces_end();
       it++) {
    CGALSurfaceMesh::Face_index fIndex(*it);
    if (segment_property_map[fIndex] == selectedSegmentIndex) {
      auto r = M.vertices_around_face(M.halfedge(fIndex));
      for (auto v : r) {
        std::size_t vIndex = std::size_t(v);
        glm::vec3 deformationVector =
            deformationFactor * vertices[vIndex].Normal;
        vertices[vIndex].Position += deformationVector;
        CGALSurfaceMesh::Point previousPosition(M.point(v));
        M.point(v) =
            CGALSurfaceMesh::Point(previousPosition.x() + deformationVector.x,
                                   previousPosition.y() + deformationVector.y,
                                   previousPosition.z() + deformationVector.z);
      }
    }
  }
}

void Mesh::setUniforms(Shader *shader) {
  shader->Use();
  modelShader = shader; // NOTE wtf is this?
  material.setUniforms(shader);
  glUniformMatrix4fv(glGetUniformLocation(shader->programID, "model"), 1,
                     GL_FALSE, glm::value_ptr(modelMatrix));
  // glUniform1f(glGetUniformLocation(shader->programID, "alpha"), alphaValue);
}

void Mesh::setIntersectingTriangleUniform(int faceIndex) {
  int location =
      glGetUniformLocation(modelShader->programID, "intersectingFace");
  glUniform1i(location, int(faceIndex));
}

void Mesh::DrawMesh() {
  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

void Mesh::loadObj(std::__cxx11::string) {}

void Mesh::setMaterial(const Material &value) { material = value; }
std::size_t
Mesh::getSegmentIndex(const CGALSurfaceMesh::Face_index face_index) const {
  return segment_property_map[face_index];
}

Ray_intersection Mesh::intersects(Kernel::Ray_3 ray) const {

  Tree tree(faces(M).first, faces(M).second, M);

  Ray_intersection intersection = tree.first_intersection(ray);
  return intersection;
}

int Mesh::findClosestVertex(Point intersectionPoint,
                            CGALSurfaceMesh::Face_index intersectingFaceIndex) {
  CGALSurfaceMesh::halfedge_index beginHalfedge =
      M.halfedge(intersectingFaceIndex);
  CGALSurfaceMesh::halfedge_index h = beginHalfedge;
  CGALSurfaceMesh::vertex_index vIndex = M.source(h);
  Kernel::Point_3 vertex = M.point(vIndex);
  double minDistance = CGAL::squared_distance(vertex, intersectionPoint);
  int closestVertexIndex = (int)vIndex;
  do {
    std::cout << h << std::endl;
    vIndex = M.source(h);
    Kernel::Point_3 vertex = M.point(vIndex);
    double distance = CGAL::squared_distance(vertex, intersectionPoint);
    if (distance < minDistance) {

      minDistance = distance;
      vIndex = M.source(h);
      closestVertexIndex = int(vIndex);
    }
    h = M.next(h);
  } while (h != beginHalfedge);
  return closestVertexIndex;
}

void Mesh::buildSurfaceMesh() {
  M.clear();
  std::cout << "Building CGALPolygonMesh.." << std::endl;
  std::vector<Kernel::Point_3> points;
  std::vector<std::vector<std::size_t>> polygons;

  for (auto const &vert : vertices) {
    points.push_back(
        Kernel::Point_3(vert.Position.x, vert.Position.y, vert.Position.z));
    // Kernel::Point_3((vert.Position.x - centerOfMass.x) / maxDim,
    //
    //                (vert.Position.y - centerOfMass.y) / maxDim,
    //                (vert.Position.z - centerOfMass.z) / maxDim));
  }
  for (size_t i = 0; i < indices.size(); i += 3) {
    std::vector<std::size_t> tri{indices[i], indices[i + 1], indices[i + 2]};
    polygons.push_back(tri);
  }
  //               CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
  std::cout << "is polygon soup polygon mesh:"
            << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
                   polygons)
            << std::endl;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons,
                                                              M);

  std::cout << "Finished building Polygon Mesh." << std::endl;
  std::cout << "Is closed:" << CGAL::is_closed(M) << std::endl;
}

void Mesh::normalizeMeshViaModelMatrix() {
  modelMatrix = glm::mat4(1.0);
  float scaleFactor = 1.0 / maxDim;
  modelMatrix = glm::scale(modelMatrix, glm::vec3(scaleFactor));

  glm::vec3 translationVector(-centerOfMass);
  modelMatrix = glm::translate(modelMatrix, translationVector);
}

void Mesh::setupDrawingBuffers() {

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MyVertex),
               &vertices[0], GL_DYNAMIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               &indices[0], GL_DYNAMIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)offsetof(MyVertex, Normal));
  glEnableVertexAttribArray(1);

  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)offsetof(MyVertex, Color));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);
  // std::cout << "printDebugInformation was called in "<<__func__<<std::endl;
  //     // printDebugInformation();
  //
}

std::vector<MyVertex> Mesh::getVertices() const { return vertices; }

void Mesh::updateMeshBuffers() {

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MyVertex),
               &vertices[0], GL_DYNAMIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               &indices[0], GL_DYNAMIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)offsetof(MyVertex, Normal));
  glEnableVertexAttribArray(1);

  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MyVertex),
                        (GLvoid *)offsetof(MyVertex, Color));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);
}

MeshSegment Mesh::getMeshSegment(size_t segmentIndex) const {
  MeshSegment S;

  std::vector<Kernel::Point_3> points;
  std::vector<std::vector<size_t>> faces;

  for (CGALSurfaceMesh::Face_iterator it = M.faces_begin(); it != M.faces_end();
       it++) {
    CGALSurfaceMesh::Face_index fIndex(*it);
    if (segment_property_map[fIndex] == segmentIndex) {
      CGALSurfaceMesh::Halfedge_index h = M.halfedge(fIndex);
      CGALSurfaceMesh::Halfedge_index hNext = M.next_around_target(h);
      CGALSurfaceMesh::Halfedge_index hPrevious = M.next_around_source(h);

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
        S.vertexCorrespondence.push_back(size_t(v1));
        i1 = points.size() - 1;
      }

      size_t i2 = std::distance(points.begin(),
                                std::find(points.begin(), points.end(), p2));
      if (i2 == points.size()) {
        points.push_back(p2);
        S.vertexCorrespondence.push_back(size_t(v2));
        i2 = points.size() - 1;
      }

      size_t i3 = std::distance(points.begin(),
                                std::find(points.begin(), points.end(), p3));
      if (i3 == points.size()) {
        points.push_back(p3);
        S.vertexCorrespondence.push_back(size_t(v3));
        i3 = points.size() - 1;
      }

      std::vector<size_t> face{i1, i2, i3};
      faces.push_back(face);
    }
  }
  S.indices = faces;
  S.vertices = points;
  std::cout << "is polygon soup polygon mesh:"
            << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
                   faces)
            << std::endl;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, faces,
                                                              S.M);
  std::cout << "Is closed:" << CGAL::is_closed(S.M) << std::endl;
  if (!CGAL::is_closed(S.M)) {
    std::cout << "Segment is not closed." << std::endl;
    std::cout << "Stitching mesh.." << std::endl;
    CGAL::Polygon_mesh_processing::stitch_borders(S.M);
  }
  std::cout << "Finished building Polygon Mesh." << std::endl;
  std::cout << "Is closed:" << CGAL::is_closed(S.M) << std::endl;
  return S;
}

void Mesh::handle_segmentSelection(Ray_intersection intersection) {
  CGALSurfaceMesh::Face_index intersectingTriangleIndex;
  intersectingTriangleIndex = intersection->second;
  size_t intersectionIndex = getSegmentIndex(intersectingTriangleIndex);

  selectedSegmentIndex = intersectionIndex;
  std::cout << "Intersection segment:" << selectedSegmentIndex << std::endl;
  colorPickedSegment();
  updateMeshBuffers();

  segment = getMeshSegment(selectedSegmentIndex);
  // SMC = MeshContractor(segment.M);
}

void Mesh::computeSegments() { constructSegmentMap(); }

void Mesh::handle_showSegments() {
  if (!segmentsComputed) {
    computeSegments();
  }
  assignSegmentColors();
  updateMeshBuffers();
}

void Mesh::inflation_handler() { // TODO δεν βρισκει τομη ακτινας με το mesh
                                 // μετα το deformation
  if (segmentHasBeenSelected()) {
    float deformationFactor = 0.01;
    inflationDeflationDeformer(deformationFactor);
    updateMeshBuffers();
  }
}

void Mesh::deflation_handler() {
  if (segmentHasBeenSelected()) {
    float deformationFactor = -0.01;
    inflationDeflationDeformer(deformationFactor);
    updateMeshBuffers();
  }
}

void Mesh::unselectSegment() { selectedSegmentIndex = -1; }

void Mesh::updateDrawingVertices() { // uses M to update std::vector<MyVertex>
                                     // vertices
  for (auto v : M.vertices()) {
    CGALSurfaceMesh::Point p = M.point(v);
    vertices[size_t(v)].Position = glm::vec3(p.x(), p.y(), p.z());
  }
}

void Mesh::handle_drawing(Shader *shader) {
  shader->Use();
  setUniforms(shader);
  DrawMesh();
}

void Mesh::handle_saveModel(std::__cxx11::string destinationPathAndFileName) {
  std::ofstream outFile;
  outFile.open(destinationPathAndFileName, std::ios::out);
  if (!outFile) {
    std::cerr << "Can't save file: " << destinationPathAndFileName << std::endl;
  } else {
    CGAL::write_off(outFile, M);
  }
  outFile.close();
}

std::vector<size_t> Mesh::getVertexIndicesWithHighLaplacianValue() {
  return MC.getVertexIndicesWithHighLaplacianValue();
}

void Mesh::handle_meshContraction() {
  if (M.has_garbage())
    M.collect_garbage();

  MC.executeContractionStep();
  M = MC.getContractedMesh();
  updateDrawingVertices(); // Wrong?
  // centerOfMass = meshMeasuring::findCenterOfMass(vertices);
  // maxDim = meshMeasuring::findMaxDimension(vertices);
  // normalizeMeshViaModelMatrix();
  updateMeshBuffers();
}

void Mesh::handle_segmentContraction() {
  SMC.executeContractionStep();
  CGALSurfaceMesh contractedSegment = SMC.getContractedMesh();
  segment.M = contractedSegment;
  updateVertices(segment);
  updateMeshBuffers();
}

void Mesh::updateVertices(const MeshSegment &copyFrom) {
  updateDrawingVertices(copyFrom);
  updateCGALSurfaceMeshVertices(copyFrom);
}

void Mesh::updateDrawingVertices(const MeshSegment &copyFrom) {
  size_t index = 0;
  for (auto v : copyFrom.M.vertices()) { // update drawing vertices in model
    CGALSurfaceMesh::Point p = copyFrom.M.point(v);
    vertices[copyFrom.vertexCorrespondence[index]].Position =
        glm::vec3(p.x(), p.y(), p.z());
    index++;
  }
}
void Mesh::updateCGALSurfaceMeshVertices(const MeshSegment &copyFrom) {
  size_t index = 0;
  for (auto v : copyFrom.M.vertices()) {
    CGALSurfaceMesh::Point p = copyFrom.M.point(v);
    CGALSurfaceMesh::vertex_index vInOriginalMesh(
        copyFrom.vertexCorrespondence[index]);
    M.point(vInOriginalMesh) = CGALSurfaceMesh::Point(p.x(), p.y(), p.z());
    index++;
  }
}

void Mesh::testVerticesConsistency() const {
  for (auto v : M.vertices()) {
    if (M.point(v).x() != vertices[size_t(v)].Position.x ||
        M.point(v).y() != vertices[size_t(v)].Position.y ||
        M.point(v).z() != vertices[size_t(v)].Position.z) {
      std::cout
          << "ERROR CGAL's mesh vertices are different than drawing vertices."
          << "vertex index=" << size_t(v) << std::endl;
      std::cout << "drawing vertex="
                << glm::to_string(vertices[size_t(v)].Position) << std::endl;
      std::cout << "drawing vertex=" << M.point(v) << std::endl;
    }
  }
}
