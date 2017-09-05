#include "mesh.h"

Mesh::Mesh() {
  for (auto &color : colorPalette) {
    color /= 255.0;
  }
}

void Mesh::resetMeshAttributes() {
  indices.clear();
  vertices.clear();
  centerOfMass = glm::vec3(1.0);
  modelMatrix = glm::mat4(1.0);
}

void Mesh::load(std::string filename) {
  resetMeshAttributes();
  std::tie(indices, vertices) =
      meshLoader::load(filename); // vertices contains coords & normals
  std::cout << "INDICES SIZE=" << indices.size() << std::endl;

  setupDrawingBuffers();
  // Find the model matrix that normalizes the mesh
  centerOfMass = meshMeasuring::findCenterOfMass(vertices);
  maxDim = meshMeasuring::findMaxDimension(vertices);
  normalizeMeshViaModelMatrix();
  buildPolygonMesh();
  // averageEdgeLength = meshMeasuring::findAverageEdgeLength(M);

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

void Mesh::printDebugInformation() const {
  glBindVertexArray(VAO);
  for (int i = 0; i <= 1; i++) {
    GLint ival;
    GLvoid *pval;

    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &ival);
    printf("Attr %d: ENABLED    		= %d\n", i, ival);
    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &ival);
    printf("Attr %d: SIZE		= %d\n", i, ival);
    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &ival);
    printf("Attr %d: STRIDE		= %d\n", i, ival);
    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &ival);
    printf("Attr %d: TYPE		= 0x%x\n", i, ival);
    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, &ival);
    printf("Attr %d: NORMALIZED		= %d\n", i, ival);
    glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &ival);
    printf("Attr %d: BUFFER		= %d\n", i, ival);
    glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pval);
    printf("Attr %d: POINTER		= %p\n", i, ival);
  }
  // Also print the numeric handle of the VAO:
  printf("VAO = %ld\n", long(VAO));
}

void Mesh::setUniforms(Shader *shader) {
  modelShader = shader; // NOTE wtf is this?
  material.setUniforms(modelShader);
  glUniformMatrix4fv(glGetUniformLocation(modelShader->programID, "model"), 1,
                     GL_FALSE, glm::value_ptr(modelMatrix));
}

void Mesh::setIntersectingTriangleUniform(int faceIndex) {
  int location =
      glGetUniformLocation(modelShader->programID, "intersectingFace");
  glUniform1i(location, int(faceIndex));
}

void Mesh::Draw() {
  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}
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

void Mesh::buildPolygonMesh() {
  M.clear();
  std::cout << "Building CGALPolygonMesh.." << std::endl;
  std::vector<Kernel::Point_3> points;
  std::vector<std::vector<std::size_t>> polygons;

  for (auto const &vert : vertices) {
    points.push_back(
        Kernel::Point_3((vert.Position.x - centerOfMass.x) / maxDim,
                        (vert.Position.y - centerOfMass.y) / maxDim,
                        (vert.Position.z - centerOfMass.z) / maxDim));
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

      CGALSurfaceMesh::Point pTemp =
          M.point(v1).transform(CGAL::Aff_transformation_3<Kernel>(
              CGAL::Scaling(), Kernel::RT(maxDim)));
      CGALSurfaceMesh::Point v1ModelCoords = CGALSurfaceMesh::Point(
          pTemp.x() + centerOfMass.x, pTemp.y() + centerOfMass.y,
          pTemp.z() + centerOfMass.z);
      pTemp = M.point(v2).transform(CGAL::Aff_transformation_3<Kernel>(
          CGAL::Scaling(), Kernel::RT(maxDim)));
      CGALSurfaceMesh::Point v2ModelCoords = CGALSurfaceMesh::Point(
          pTemp.x() + centerOfMass.x, pTemp.y() + centerOfMass.y,
          pTemp.z() + centerOfMass.z);
      pTemp = M.point(v3).transform(CGAL::Aff_transformation_3<Kernel>(
          CGAL::Scaling(), Kernel::RT(maxDim)));
      CGALSurfaceMesh::Point v3ModelCoords = CGALSurfaceMesh::Point(
          pTemp.x() + centerOfMass.x, pTemp.y() + centerOfMass.y,
          pTemp.z() + centerOfMass.z);

      size_t i1 =
          std::distance(points.begin(),
                        std::find(points.begin(), points.end(), v1ModelCoords));
      if (i1 == points.size()) {
        points.push_back(v1ModelCoords);
        S.vertexCorrespondence.push_back(size_t(v1));
        i1 = points.size() - 1;
      }

      size_t i2 =
          std::distance(points.begin(),
                        std::find(points.begin(), points.end(), v2ModelCoords));
      if (i2 == points.size()) {
        points.push_back(v2ModelCoords);
        S.vertexCorrespondence.push_back(size_t(v2));
        i2 = points.size() - 1;
      }

      size_t i3 =
          std::distance(points.begin(),
                        std::find(points.begin(), points.end(), v3ModelCoords));
      if (i3 == points.size()) {
        points.push_back(v3ModelCoords);
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

void Mesh::handleSegmentSelection(Ray_intersection intersection) {
  CGALSurfaceMesh::Face_index intersectingTriangleIndex;
  intersectingTriangleIndex = intersection->second;
  size_t intersectionIndex = getSegmentIndex(intersectingTriangleIndex);

  selectedSegmentIndex = intersectionIndex;
  std::cout << "Intersection segment:" << selectedSegmentIndex << std::endl;
  colorPickedSegment();
  updateMeshBuffers();
}

void Mesh::computeSegments() { constructSegmentMap(); }

void Mesh::handleShowSegments() {
  if (!segmentsComputed) {
    computeSegments();
  }
  assignSegmentColors();
  updateMeshBuffers();
}

void Mesh::updateVertices(
    const CGALSurfaceMesh &copyFrom) // NOTE is copyFrom a bad name?
{
  updateDrawingVertices(copyFrom);
  updateCGALSurfaceMeshVertices(copyFrom);
  updateMeshBuffers();
  // testVerticesConsistency();
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

void Mesh::updateCGALSurfaceMeshVertices(const CGALSurfaceMesh &copyFrom) {
  for (auto v : copyFrom.vertices()) {
    CGALSurfaceMesh::Point p = copyFrom.point(v);
    M.point(v) = CGALSurfaceMesh::Point((p.x() - centerOfMass.x) / maxDim,
                                        (p.y() - centerOfMass.y) / maxDim,
                                        (p.z() - centerOfMass.z) / maxDim);
  }
}

void Mesh::updateDrawingVertices(const CGALSurfaceMesh &copyFrom) {
  for (auto v : copyFrom.vertices()) { // update drawing vertices in model
    CGALSurfaceMesh::Point p = copyFrom.point(v);
    vertices[size_t(v)].Position = glm::vec3(p.x(), p.y(), p.z());
  }

  centerOfMass = meshMeasuring::findCenterOfMass(vertices);
  maxDim = meshMeasuring::findMaxDimension(vertices);
  normalizeMeshViaModelMatrix();
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
