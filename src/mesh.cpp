#include "mesh.h"
#include <thread>

// void Mesh::setPointSphere(PointSphere ps) { m_PS = ps; }
void Mesh::handle_showVerticesStateChange(int state) {
  m_showPointSpheresOnVertices = bool(state);
}

void Mesh::handle_segmentSkeletonization() {}

// void Mesh::handle_skeletonization() {
//	if (!CGAL::is_triangle_mesh(m_M)) {
//		std::cerr << "Input geometry is not triangulated." << std::endl;
//	} else {
//		using CGALSkeletonization =
//		    CGAL::Mean_curvature_flow_skeletonization<CGALSurfaceMesh>;
//		using CGALSkeleton = CGALSkeletonization::Skeleton;
//		using CGALSkeleton_vertex = CGALSkeleton::vertex_descriptor;
//		using CGALSkeleton_edge = CGALSkeleton::edge_descriptor;
//
//		CGALSkeleton s;
//
//		// CGAL::extract_mean_curvature_flow_skeleton(m_M, s);
//
//		auto it_pair = boost::edges(s);
//
//		std::vector<std::vector<size_t>> edges;
//		for (auto it = it_pair.first; it != it_pair.second; it++) {
//			CGALSkeleton::vertex_descriptor vdS =
//			    boost::source(*it, s);
//			CGALSkeleton::vertex_descriptor vdT =
//			    boost::target(*it, s);
//			std::vector<size_t> e{size_t(vdS), size_t(vdT)};
//			edges.push_back(e);
//		}
//
//		std::vector<CGALSurfaceMesh::Point> nodes;
//		BOOST_FOREACH (CGALSkeleton_vertex v, vertices(s)) {
//			nodes.push_back(s[v].point);
//		}
//
//		std::cout << "Number of edges:" << edges.size() << std::endl;
//		std::cout << "Number of nodes:" << nodes.size() << std::endl;
//
//		Skeleton skeleton;
//		skeleton.append(edges, nodes, m_PS);
//		m_skeleton.addSkeletonPart(skeleton);
//	}
//}

void Mesh::segmentMesh() {
  if (!segmentsComputed) {
    size_t numberOfSegments = computeSegments();
    // std::cout << "Constructing segment graph.." << std::endl;
    constructSegmentGraph(numberOfSegments); // m_segmentGraph
    // m_perSegmentSkeletonEdges.resize(numberOfSegments);

    m_skeleton.setNumberOfSkeletonParts(numberOfSegments);
    // old skeleton
    // m_skeleton.setNumberOfSegments(numberOfSegments);
    segmentsComputed = true;
  }
}

void Mesh::skeletonize() {
  handle_meshContraction(true);
  handle_meshConnectivitySurgery();
}

CGALSurfaceMesh::halfedge_index
getNextHalfedgeInLoop(CGALSurfaceMesh M, CGALSurfaceMesh::halfedge_index h) {
  for (CGALSurfaceMesh::halfedge_index hi :
       CGAL::halfedges_around_source(M.target(h), M)) {
    if (M.is_border(hi))
      return hi;
  }
}
std::vector<std::vector<size_t>> getMeshLoops(const CGALSurfaceMesh &M) {
  std::vector<std::vector<size_t>>
      meshLoops; // vertex indices grouped based on the loops they belong
  std::vector<bool> was_processed(M.number_of_halfedges(), false);
  for (CGALSurfaceMesh::halfedge_index hi : M.halfedges()) {
    if (was_processed[size_t(hi)])
      continue;
    was_processed[size_t(hi)] = true;

    if (M.is_border(hi)) {
      std::vector<size_t> verticesInLoop{size_t(M.source(hi))};
      CGALSurfaceMesh::halfedge_index hLoop = getNextHalfedgeInLoop(M, hi);
      was_processed[size_t(hLoop)] = true;
      while (hLoop != hi) {
        verticesInLoop.push_back(size_t(M.source(hLoop)));
        hLoop = getNextHalfedgeInLoop(M, hLoop);
        was_processed[size_t(hLoop)] = true;
      }
      meshLoops.push_back(verticesInLoop);
    }
  }

  return meshLoops;
}

template <typename BoostGraph>
std::vector<std::set<typename BoostGraph::vertex_descriptor>>
extractPossibleJunctionNodes(BoostGraph skeleton,
                             std::vector<std::vector<size_t>> meshLoops,
                             size_t numOfVerts) {
  std::vector<typename BoostGraph::vertex_descriptor>
      vertex_to_collapsedSkeletonNode(numOfVerts);
  std::vector<std::set<typename BoostGraph::vertex_descriptor>>
      possibleBoarderSkeletonNodeGroups;
  BOOST_FOREACH (typename BoostGraph::vertex_descriptor vd,
                 boost::vertices(skeleton)) {
    for (size_t collapseVertexIndex : skeleton[vd].collapsed_vertices) {
      vertex_to_collapsedSkeletonNode[collapseVertexIndex] = vd;
    }
  }
  for (std::vector<size_t> loopVertices : meshLoops) {
    std::set<typename BoostGraph::vertex_descriptor> loopPossibleSkeletonNodes;
    for (size_t loopVertex : loopVertices) {
      loopPossibleSkeletonNodes.insert(
          vertex_to_collapsedSkeletonNode[loopVertex]);
    }
    possibleBoarderSkeletonNodeGroups.push_back(loopPossibleSkeletonNodes);
  }
  return possibleBoarderSkeletonNodeGroups;
}

void Mesh::handle_segmentConnectivitySurgery() {
  assert(selectedSegmentIndex);
  std::vector<std::vector<size_t>> meshLoops =
      getMeshLoops(m_segment.getOriginalMesh());
  CGALSurfaceMesh contractedSegment = m_segment.getMesh();
  // std::cout << "Original segment has "
  //	  << m_segment.getOriginalMesh().number_of_vertices()
  //	  << " vertices." << std::endl;
  ConnectivitySurgeon<CGALSurfaceMesh> CS(contractedSegment,
                                          m_segment.getOriginalMesh());
  auto skeletonGraph = CS.execute_connectivitySurgery();
  std::vector<std::set<SkeletonGraph::vertex_descriptor>>
      possibleBoarderSkeletonNodeGroups = extractPossibleJunctionNodes(
          skeletonGraph, meshLoops, m_M.number_of_vertices());

  SkeletonPart sp(skeletonGraph, possibleBoarderSkeletonNodeGroups,
                  selectedSegmentIndex.get());

  m_skeleton.addSkeletonPart(sp);
  // SkeletonRefinement::refine(skeletonGraph,
  // m_segment.getOriginalMesh());

  m_showContractedSegment = false;
}

#include <chrono>
void Mesh::MySkeletonization(unsigned int segmentIndex) {
  // SEGMENT CONTRACTION
  // auto contractionTimeStart =
  // std::chrono::high_resolution_clock::now();
  // m_segment.changeSegment(segmentIndex);
  // CGALSurfaceMesh originalSegment = m_segment.getMesh();
  CGALSurfaceMesh originalSegment =
      constructSelectedSegmentSurfaceMesh(segmentIndex);
  MeshContractor SMC = MeshContractor(originalSegment);
  // segmentContractionIterationsCounter += SMC.contractMesh();
  SMC.contractMesh();
  // auto contractionTimeEnd = std::chrono::high_resolution_clock::now();
  // totalContractionTime += std::chrono::duration<double>(
  //			    contractionTimeEnd - contractionTimeStart)
  //			    .count();

  // SEGMENT CONNECTIVITY SURGERY
  // auto connectivitySurgeryTimeStart =
  //    std::chrono::high_resolution_clock::now();
  CGALSurfaceMesh contractedSegment = SMC.getContractedMesh();
  ConnectivitySurgeon<CGALSurfaceMesh> CS(contractedSegment, originalSegment);
  auto skeletonGraph = CS.execute_connectivitySurgery();
  // auto connectivitySurgeryTimeEnd =
  //    std::chrono::high_resolution_clock::now();
  // totalConnectivitySurgeryTime +=
  //    std::chrono::duration<double>(connectivitySurgeryTimeEnd -
  //				  connectivitySurgeryTimeStart)
  //	.count();

  // FIND POSSIBLE JUNCTION NODES WITH THE REST OF THE SKELETON
  // auto junctionNodeComputationTimeStart =
  //    std::chrono::high_resolution_clock::now();
  std::vector<std::vector<size_t>> segmentHoleVertices =
      getMeshLoops(originalSegment);
  std::vector<std::set<SkeletonGraph::vertex_descriptor>>
      possibleJunctionNodes = extractPossibleJunctionNodes(
          skeletonGraph, segmentHoleVertices, m_M.number_of_vertices());
  // auto junctionNodeComputationTimeEnd =
  //    std::chrono::high_resolution_clock::now();
  // totalJunctionNodeComputationTime +=
  //    std::chrono::duration<double>(junctionNodeComputationTimeEnd -
  //				  junctionNodeComputationTimeStart)
  //	.count();

  // ADD SKELETON PART TO SKELETON
  // auto skeletonPartAdditionTimeStart =
  //    std::chrono::high_resolution_clock::now();
  SkeletonPart sp(skeletonGraph, possibleJunctionNodes, segmentIndex);

  m_skeleton.addSkeletonPart(sp);
  // auto skeletonPartAdditionTimeEnd =
  //    std::chrono::high_resolution_clock::now();
  // totalSkeletonPartAdditionTime +=
  //    std::chrono::duration<double>(skeletonPartAdditionTimeEnd -
  //				  skeletonPartAdditionTimeStart)
  //	.count();
  // SkeletonRefinement::refine(skeletonGraph,
  // m_segment.getOriginalMesh());

  // handle_segmentConnectivitySurgery();
  std::cout << "finished segment skeletonization for segmentIndex="
            << segmentIndex << std::endl;
}

void Mesh::serialSkeletonizationUsingSegments() {
  for (size_t i = 0; i < m_numberOfSegments; i++) {
    MySkeletonization(i);
  }
  m_skeleton.connectNeighbourSegments();
}
void Mesh::fillMeshHoles() {
  std::cout << "MESH IS NOT WATERTIGHT!In order to use skeletonization "
               "via segmenation the model needs to be "
               "watertight.Filling "
               "mesh holes.."
            << std::endl;
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
void Mesh::parallelSkeletonization() {
  if (!CGAL::is_closed(m_M)) {
    fillMeshHoles();
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  segmentMesh();
  std::cout << "Skeletonizing segments and threads.." << std::endl;
  std::vector<std::thread> threads(m_numberOfSegments);

  for (size_t i = 0; i < m_numberOfSegments; i++) {
    threads[i] = std::thread(&Mesh::MySkeletonization, this, i);
  }
  for (size_t i = 0; i < m_numberOfSegments; i++) {
    threads[i].join();
  }
  m_skeleton.connectNeighbourSegments();
  auto t2 = std::chrono::high_resolution_clock::now();
  double parallelSkeletonizationTime =
      std::chrono::duration<double>(t2 - t1).count();
  std::cout << "Parallel skeletonization took:" << parallelSkeletonizationTime
            << " sec" << std::endl;
  alphaValue = 0.2;
  m_showContractedSegment = false;
}
void Mesh::runSkeletonizationMethods() {
  if (!CGAL::is_closed(m_M)) {
    std::cout << "MESH IS NOT WATERTIGHT!In order to use skeletonization "
                 "via segmenation the model needs to be "
                 "watertight.Filling "
                 "mesh holes.."
              << std::endl;
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
  std::cout << "Segmenting mesh.." << std::endl;
  auto segmentationTimeStart = std::chrono::high_resolution_clock::now();
  segmentMesh();
  auto segmentationTimeEnd = std::chrono::high_resolution_clock::now();
  double segmentationTime =
      std::chrono::duration<double>(segmentationTimeEnd - segmentationTimeStart)
          .count();
  std::cout << "Segmentation time:" << segmentationTime << " sec" << std::endl;

  // std::string modelName(filename.begin() + filename.find_last_of("/"),
  //		      filename.end());
  // modelName.erase(modelName.begin() + modelName.find("."),
  //		modelName.end());
  // std::string destination = "../../Measurements" + modelName + ".txt";
  // std::ofstream myfile(destination);
  //// std::cout << "Destination:" << destination << std::endl;
  // myfile << "Vertices:" << m_M.number_of_vertices() << "\n"
  //       << "segmentation time:"
  //       << std::chrono::duration<double>(segmentationTimeEnd -
  //					segmentationTimeStart)
  //	      .count()
  //       << "\n";

  parallelSkeletonization();

  auto t1 = std::chrono::high_resolution_clock::now();
  serialSkeletonizationUsingSegments();
  auto t2 = std::chrono::high_resolution_clock::now();
  double serialSkeletonizationUsingSegmentsTime =
      std::chrono::duration<double>(t2 - t1).count();
  std::cout << "skeletonization without using threads took:"
            << serialSkeletonizationUsingSegmentsTime << " sec" << std::endl;

  // MESH SKELETONIZATION WITHOUT SEGMENTS AND THREADS
  t1 = std::chrono::high_resolution_clock::now();
  std::cout << "Skeletonizing using the classic approach.." << std::endl;
  handle_meshContraction(true);
  handle_meshConnectivitySurgery();
  t2 = std::chrono::high_resolution_clock::now();
  double noSegmentsNoThreadsTime =
      std::chrono::duration<double>(t2 - t1).count();
  std::cout << "Skeletonization using the classic approach took:"
            << noSegmentsNoThreadsTime << " sec" << std::endl;
  // myfile << "whole mesh contraction number of iterations:"
  //       << wholeMeshContractionNumberOfIterations << std::endl;
  // myfile << "total contraction time without using segments:"
  //       << std::chrono::duration<double>(meshContractionTimeEnd -
  //					meshContractionTimeStart)
  //	      .count()
  //       << std::endl;
  // std::cout << "total contraction time without using segments:"
  //	  << std::chrono::duration<double>(meshContractionTimeEnd -
  //					   meshContractionTimeStart)
  //		 .count()
  //	  << std::endl;
  // myfile.close();
  alphaValue = 0.2;
  m_showContractedSegment = false;
}

void Mesh::resetMeshAttributes() {
  m_indices.clear();
  m_vertices.clear();
  pointSphereDrawingVector.clear();
  centerOfMass = glm::vec3(0.0);
  m_modelMatrix = glm::mat4(1.0);
  segmentsComputed = false;
  m_showContractedSegment = false;
  m_skeleton.clear();
  alphaValue = 1;
}

void Mesh::load(std::string filename) {
  this->filename = filename;
  resetMeshAttributes();
  try {
    populateVerticesAndIndices(filename);
  } catch (std::string error) {
    std::cerr << error << std::endl;
  }

  normalizeMeshViaModelMatrix();
  initializeDrawingBuffers();

  m_skeleton.initialize();
  m_segment.initialize();

  createCGALSurfaceMesh();
  m_originalMesh = m_M;
  // MC = MeshContractor(M, indices);
  MC = MeshContractor(m_M);
  // computeSegments();

  printMeshInformation();
}

void Mesh::assignSegmentColors() {
  for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
    std::size_t segmentIndex = m_segmentFaceMap[fIndex];
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
  // std::cout << "Segmenting mesh.." << std::endl;
  // create a property-map ean are you sure the edge sean are you sure the
  // edge shared by two triangles is not duplicatehared by two triangles
  // is
  // not duplicatefor segment-id
  m_segmentFaceMap =
      m_M.add_property_map<face_descriptor, std::size_t>("f:sid").first;
  ;
  segmentsComputed = true;
  // segment the mesh using default parameters for number of levels, and
  // smoothing lambda
  // Any other scalar values can be used instead of using SDF values
  // computed
  // using the CGAL function
  m_numberOfSegments = CGAL::segmentation_via_sdf_values(
      // m_M, m_segmentFaceMap, 2.0 * CGAL_PI / 3, 25, 3, 0.4);
      // m_M, m_segmentFaceMap, 2.0 * CGAL_PI / 3, 25, 4,0.5);
      m_M, m_segmentFaceMap, 2.0 * CGAL_PI / 3, 25, 3, 0.4);

  // std::cout << "Number of segments: " << m_numberOfSegments <<
  // std::endl;
  return m_numberOfSegments;
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

void Mesh::handle_drawing(Shader *activeModelShader, Shader *edgeShader,
                          Shader *nodeShader) {
  m_skeleton.Draw(edgeShader, nodeShader, m_modelMatrix);

  if (m_showContractedSegment) {
    m_segment.handle_drawing(activeModelShader);
  }

  drawThisMesh(activeModelShader);

  // if (m_showPointSpheresOnVertices) {
  // // draw specific vertices
  // for (PointSphere s : pointSphereDrawingVector) {
  // 	s.handle_drawing(modelShader, m_modelMatrix);
  // 	}
  // }
  // if (m_showLaplacianHeatMap) {
  // 	for (PointSphere ps : m_laplacianHeatMap) {
  // 		ps.handle_drawing(modelShader, m_modelMatrix);
  // 	}
  // }
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

void Mesh::populateVerticesAndIndices(std::string filename) {
  std::tie(m_indices, m_vertices) =
      meshLoader::load(filename); // vertices contains coords & normals
  if (m_indices.size() == 0 || m_vertices.size() == 0)
    throw std::string("ERROR:File " + filename + " could not be loaded!");
}

std::size_t Mesh::getCorrespondingSegmentIndex(
    const CGALSurfaceMesh::Face_index face_index) const {
  return m_segmentFaceMap[face_index];
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

// MeshSegment Mesh::getMeshSegment() const {
// assert(selectedSegmentIndex);
// MeshSegment S(m_M, m_vertices, m_segmentFaceMap, selectedSegmentIndex.get(),
// 	      m_modelMatrix);
// return S;
// }

void Mesh::handle_segmentSelection(Ray_intersection intersection) {
  CGALSurfaceMesh::Face_index intersectingFaceIndex = intersection->second;
  selectedSegmentIndex = getCorrespondingSegmentIndex(intersectingFaceIndex);

  // std::cout << "Intersection segment:" << selectedSegmentIndex.get()
  //<< std::endl;

  colorPickedSegment();
  updateMeshBuffers();

  m_segment.changeSegment(selectedSegmentIndex.get());
  SMC = MeshContractor(m_segment.getMesh());
}

size_t Mesh::computeSegments() {
  segmentsComputed = true;
  size_t numberOfSegments = constructSegmentMap();
  return numberOfSegments;
}

void Mesh::handle_showSegments() {
  segmentMesh();

  // std::cout << "Assigning colors.." << std::endl;
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

void Mesh::handle_saveModel(std::string destinationPathAndFileName) {
  std::ofstream outFile;
  outFile.open(destinationPathAndFileName, std::ios::out);
  if (!outFile) {
    std::cerr << "Can't save file: " << destinationPathAndFileName << std::endl;
  } else {
    if (!CGAL::write_off(outFile, m_M))
      std::cerr << "Can't save file: " << destinationPathAndFileName
                << std::endl;
  }
  outFile.close();
}

void Mesh::handle_saveSegment(std::string destinationPathAndFileName) {
  std::ofstream outFile;
  outFile.open(destinationPathAndFileName, std::ios::out);
  if (!outFile) {
    std::cerr << "Can't save file: " << destinationPathAndFileName << std::endl;
  } else {
    CGAL::write_off(outFile, m_segment.getOriginalMesh());
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

  updatePointSphereDrawingVector();
}

void Mesh::handle_meshContraction(bool automatic) {
  if (m_M.has_garbage())
    m_M.collect_garbage();

  if (automatic)
    MC.contractMesh();
  else {
    MC.executeContractionStep();
    // MC.executeCGALContraction();
  }
  m_M = MC.getContractedMesh();

  DrawableMesh::updateDrawingVertices();
  updateMeshBuffers();

  // updatePointSphereDrawingVector();
}

void Mesh::handle_segmentContraction(bool automatic) {
  alphaValue = 0.4;
  if (automatic)
    SMC.contractMesh();
  else
    SMC.executeContractionStep();
  CGALSurfaceMesh contractedSegment = SMC.getContractedMesh();
  m_segment.setMesh(contractedSegment);
  m_showContractedSegment = true;
}

// Skeleton Mesh::convertToSkeleton(
//    std::vector<std::vector<size_t>> skeletonEdgesInMeshIndices,
//    const CGALSurfaceMesh &M) const {
//	Skeleton s;
//	s.initialize();
//	using MeshIndex = size_t;
//	using SkeletonIndex = size_t;
//	std::unordered_set<size_t> skeletonNodesInMeshIndices;
//
//	for (auto skeletonEdge : skeletonEdgesInMeshIndices) {
//		for (auto v : skeletonEdge) {
//			skeletonNodesInMeshIndices.insert(v);
//		}
//	}
//	std::vector<CGALSurfaceMesh::Point> skeletonNodePositions;
//	std::map<MeshIndex, SkeletonIndex> meshIndices_to_skeletonIndices;
//	SkeletonIndex indexInSkeleton = 0;
//	for (MeshIndex indexInMesh : skeletonNodesInMeshIndices) {
//		// skeletonNodePositions.push_back(
//		//    CGALSurfaceMesh::Point(1 + indexInMesh, 2 + indexInMesh,
//		//    0));
//		skeletonNodePositions.push_back(
//		    M.point(CGALSurfaceMesh::vertex_index(indexInMesh)));
//		meshIndices_to_skeletonIndices.insert(
//		    std::make_pair(indexInMesh, indexInSkeleton));
//		indexInSkeleton++;
//	}
//
//	// skeletonEdges with mesh indices -> skeletonEdges with skeleton
//	// indices
//
//	std::vector<std::vector<size_t>> skeletonEdgesInSkeletonIndices;
//	for (auto edge : skeletonEdgesInMeshIndices) {
//		skeletonEdgesInSkeletonIndices.push_back(std::vector<size_t>{
//		    meshIndices_to_skeletonIndices[edge[0]],
//		    meshIndices_to_skeletonIndices[edge[1]]});
//	}
//
//	s.append(skeletonEdgesInSkeletonIndices, skeletonNodePositions, m_PS);
//	for (auto edge : skeletonEdgesInSkeletonIndices) {
//		for (auto v : edge)
//			std::cout << "vertex index " << v << std::endl;
//	}
//	return s;
//}
//
void Mesh::constructSegmentGraph(size_t numberOfSegments) {
  m_segmentGraph = UndirectedGraph(numberOfSegments);

  for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
    size_t fSegmentIndex = getCorrespondingSegmentIndex(fIndex);

    for (CGALSurfaceMesh::face_index neighbouringFaceIndex :
         m_M.faces_around_face(m_M.halfedge(fIndex))) {
      if (neighbouringFaceIndex ==
          boost::graph_traits<CGALSurfaceMesh>::null_face())
        continue;

      size_t neigFSegmentIndex =
          getCorrespondingSegmentIndex(neighbouringFaceIndex);

      if (fSegmentIndex != neigFSegmentIndex) {
        // bool successfullyAdded =
        m_segmentGraph.add_edge(fSegmentIndex, neigFSegmentIndex);
        // assert(successfullyAdded);
      }
    }
  }
  // std::cout << "Segment graph has:" << m_segmentGraph.num_vertices()
  //		  << " nodes" << std::endl;
  //	std::cout << "Segment graph has:" << m_segmentGraph.num_edges()
  //<< " edges" << std::endl;
}

void Mesh::updatePointSphereDrawingVector() {
  pointSphereDrawingVector.clear();
  for (size_t vi : MC.getLowOneRingAreaVertices()) {
    PointSphere ps = m_PS;
    ps.setPosition(m_M.point(CGALSurfaceMesh::vertex_index(vi)));
    ps.setColor(glm::vec3(1, 0, 0));
    pointSphereDrawingVector.push_back(ps);
  }
  for (size_t vi : MC.getHighOneRingAreaVertices()) {
    PointSphere ps = m_PS;
    ps.setPosition(m_M.point(CGALSurfaceMesh::vertex_index(vi)));
    ps.setColor(glm::vec3(0, 1, 0));
    pointSphereDrawingVector.push_back(ps);
  }
}

void Mesh::handle_meshConnectivitySurgery() {
  const CGALSurfaceMesh &contractedMesh = m_M;
  ConnectivitySurgeon<CGALSurfaceMesh> CS(contractedMesh, m_originalMesh);
  auto skeletonGraph = CS.execute_connectivitySurgery();
  // SkeletonRefinement::refine(skeletonGraph, m_originalMesh);

  // std::vector<Edge> skeletonEdges = CS.getSkeletonEdges();

  // auto skeletonNodes = CS.getSkeletonNodes();
  // m_skeletonMeshMapping = CS.getSkeletonMeshMapping();

  // m_skeleton.populateSkeleton(skeletonEdges, skeletonNodes);
  m_skeleton.populateSkeleton(skeletonGraph);
  // m_skeleton.addSkeletonPart(
  //    convertToSkeleton(skeletonEdgesInMeshIndices, m_M));
}

void Mesh::handle_meshRefinementEmbedding() {
  // std::cout << "Embedding Refinement.." << std::endl;

  Kernel::Vector_3 node(0, 0, 0);
  std::vector<Kernel::Vector_3> points{Kernel::Vector_3(0.5, 0.5, 0.5),
                                       Kernel::Vector_3(0, 0, 0)};

  double nc = SkeletonRefinement::computeNodeCenterness(node, points);
  // std::cout << "Node centerness:" << nc << std::endl;

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

// CGALSurfaceMesh::Point
// Mesh::computeCenterOfMass(std::vector<size_t> vertexIndices) {
//  Kernel::Vector_3 sum(CGAL::NULL_VECTOR);
//  for (size_t vIndex : vertexIndices) {
//    CGALSurfaceMesh::Point p =
//    m_M.point(CGALSurfaceMesh::vertex_index(vIndex));
//    sum += Kernel::Vector_3(p.x(), p.y(), p.z());
//  }
//  Kernel::Vector_3 centerOfMassVector = sum / vertexIndices.size();
//  CGALSurfaceMesh::Point centerOfMass(
//      centerOfMassVector.x(), centerOfMassVector.y(), centerOfMassVector.z());
//  return centerOfMass;
//}

void Mesh::updateDrawingVertices(const MeshSegment &copyFrom) {
  size_t index = 0;
  for (auto v :
       copyFrom.getMesh().vertices()) { // update drawing vertices in model
    CGALSurfaceMesh::Point p = copyFrom.getMesh().point(v);
    m_vertices[copyFrom.vertexCorrespondence[index]].Position =
        glm::vec3(p.x(), p.y(), p.z());
    index++;
  }
}

CGALSurfaceMesh
Mesh::constructSelectedSegmentSurfaceMesh(unsigned int segmentIndex) const {
  auto M = m_M;
  const auto &segmentMap = m_segmentFaceMap;
  std::vector<Kernel::Point_3> points;
  std::vector<std::vector<size_t>> faces;
  for (CGALSurfaceMesh::Face_iterator it = M.faces_begin(); it != M.faces_end();
       it++) {
    CGALSurfaceMesh::Face_index fIndex(*it);
    if (segmentMap[fIndex] == segmentIndex) {
      CGALSurfaceMesh::Halfedge_index h = M.halfedge(fIndex);
      // CGALSurfaceMesh::Halfedge_index hNext =
      // M.next_around_target(h);
      // CGALSurfaceMesh::Halfedge_index hPrevious =
      // M.next_around_source(h);

      // CGALSurfaceMesh::Edge_index e1 = M.edge(h);
      // CGALSurfaceMesh::Edge_index e2 =
      // M.edge(hNext);
      // CGALSurfaceMesh::Edge_index e3 =
      // M.edge(hPrevious);

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
        i1 = points.size() - 1;
      }

      size_t i2 = std::distance(points.begin(),
                                std::find(points.begin(), points.end(), p2));
      if (i2 == points.size()) {
        points.push_back(p2);
        i2 = points.size() - 1;
      }

      size_t i3 = std::distance(points.begin(),
                                std::find(points.begin(), points.end(), p3));
      if (i3 == points.size()) {
        points.push_back(p3);
        i3 = points.size() - 1;
      }

      std::vector<size_t> face{i1, i2, i3};
      faces.push_back(face);
    }
  }
  CGALSurfaceMesh outputMesh;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, faces,
                                                              outputMesh);
  return outputMesh;
}
