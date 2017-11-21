#include "mesh.h"

void Mesh::setPointSphere(PointSphere ps) { m_PS = ps; }

void Mesh::handle_showVerticesStateChange(int state) {
	m_showPointSpheresOnVertices = bool(state);
}

void Mesh::handle_segmentSkeletonization() {}

void Mesh::handle_skeletonization() {
	if (!CGAL::is_triangle_mesh(m_M)) {
		std::cerr << "Input geometry is not triangulated." << std::endl;
	} else {
		using CGALSkeletonization =
		    CGAL::Mean_curvature_flow_skeletonization<CGALSurfaceMesh>;
		using CGALSkeleton = CGALSkeletonization::Skeleton;
		using CGALSkeleton_vertex = CGALSkeleton::vertex_descriptor;
		using CGALSkeleton_edge = CGALSkeleton::edge_descriptor;

		CGALSkeleton s;

		// CGAL::extract_mean_curvature_flow_skeleton(m_M, s);

		auto it_pair = boost::edges(s);

		std::vector<std::vector<size_t>> edges;
		for (auto it = it_pair.first; it != it_pair.second; it++) {
			CGALSkeleton::vertex_descriptor vdS =
			    boost::source(*it, s);
			CGALSkeleton::vertex_descriptor vdT =
			    boost::target(*it, s);
			std::vector<size_t> e{size_t(vdS), size_t(vdT)};
			edges.push_back(e);
		}

		std::vector<CGALSurfaceMesh::Point> nodes;
		BOOST_FOREACH (CGALSkeleton_vertex v, vertices(s)) {
			nodes.push_back(s[v].point);
		}

		std::cout << "Number of edges:" << edges.size() << std::endl;
		std::cout << "Number of nodes:" << nodes.size() << std::endl;

		Skeleton skeleton;
		skeleton.append(edges, nodes, m_PS);
		m_skeleton.addSkeletonPart(skeleton);
	}
}

void Mesh::handle_laplacianHeatMapStateChange(bool state) {
	m_showLaplacianHeatMap = state;
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
	resetMeshAttributes();
	populateVerticesAndIndices(filename);
	normalizeMeshViaModelMatrix();
	initializeDrawingBuffers();

	m_skeleton.initialize();
	segment.initialize();

	createCGALSurfaceMesh();
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
	std::cout << "Segmenting mesh.." << std::endl;
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
	std::size_t number_of_segments = CGAL::segmentation_via_sdf_values(
	    m_M, m_segmentFaceMap, 2.0 * CGAL_PI / 3, 25, 3, 0.4);

	std::cout << "Number of segments: " << number_of_segments << std::endl;
	return number_of_segments;
}

void Mesh::inflationDeflationDeformer(float deformationFactor) {
	for (CGALSurfaceMesh::face_index fIndex : m_M.faces()) {
		if (getCorrespondingSegmentIndex(fIndex) ==
		    selectedSegmentIndex.get()) {
			for (CGALSurfaceMesh::vertex_index vi :
			     m_M.vertices_around_face(m_M.halfedge(fIndex))) {
				glm::vec3 deformationVector =
				    deformationFactor *
				    m_vertices[size_t(vi)].Normal;
				m_vertices[size_t(vi)].Position +=
				    deformationVector;
				CGALSurfaceMesh::Point previousPosition(
				    m_M.point(vi));
				m_M.point(vi) = CGALSurfaceMesh::Point(
				    previousPosition.x() + deformationVector.x,
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

void Mesh::handle_drawing(Shader *modelShader, Shader *edgeShader) {
	m_skeleton.Draw(edgeShader, modelShader, m_modelMatrix);

	if (m_showContractedSegment) {
		segment.handle_drawing(modelShader);
	}

	drawThisMesh(modelShader);

	if (m_showPointSpheresOnVertices) {
		// draw specific vertices
		for (PointSphere s : pointSphereDrawingVector) {
			s.handle_drawing(modelShader, m_modelMatrix);
		}
	}
	if (m_showLaplacianHeatMap) {
		for (PointSphere ps : m_laplacianHeatMap) {
			ps.handle_drawing(modelShader, m_modelMatrix);
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
	glUniform1f(glGetUniformLocation(shader->programID, "alpha"),
		    alphaValue);
}

void Mesh::populateVerticesAndIndices(std::__cxx11::string filename) {
	std::tie(m_indices, m_vertices) =
	    meshLoader::load(filename);  // vertices contains coords & normals
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

MeshSegment Mesh::getMeshSegment() const {
	assert(selectedSegmentIndex);
	MeshSegment S(m_M, m_vertices, m_segmentFaceMap,
		      selectedSegmentIndex.get(), m_modelMatrix);
	return S;
}

void Mesh::handle_segmentSelection(Ray_intersection intersection) {
	CGALSurfaceMesh::Face_index intersectingFaceIndex =
	    intersection->second;
	selectedSegmentIndex =
	    getCorrespondingSegmentIndex(intersectingFaceIndex);

	std::cout << "Intersection segment:" << selectedSegmentIndex.get()
		  << std::endl;

	colorPickedSegment();
	updateMeshBuffers();

	segment.setSegment(getMeshSegment());
	SMC = MeshContractor(segment.M());
}

size_t Mesh::computeSegments() {
	segmentsComputed = true;
	size_t numberOfSegments = constructSegmentMap();
	return numberOfSegments;
}

void Mesh::handle_showSegments() {
	if (!segmentsComputed) {
		size_t numberOfSegments = computeSegments();
		constructSegmentGraph(numberOfSegments);  // m_segmentGraph
		// m_perSegmentSkeletonEdges.resize(numberOfSegments);
		m_skeleton.setNumberOfSegments(numberOfSegments);
	}
	alphaValue = 1;
	assignSegmentColors();
	updateMeshBuffers();
	m_showContractedSegment = false;
}

void Mesh::inflation_handler() {  // TODO δεν βρισκει τομη ακτινας με το mesh
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

void Mesh::handle_saveModel(std::__cxx11::string destinationPathAndFileName) {
	std::ofstream outFile;
	outFile.open(destinationPathAndFileName, std::ios::out);
	if (!outFile) {
		std::cerr << "Can't save file: " << destinationPathAndFileName
			  << std::endl;
	} else {
		if (!CGAL::write_off(outFile, m_M))
			std::cerr
			    << "Can't save file: " << destinationPathAndFileName
			    << std::endl;
	}
	outFile.close();
}

void Mesh::handle_saveSegment(std::__cxx11::string destinationPathAndFileName) {
	MeshSegment segment = getMeshSegment();
	std::ofstream outFile;
	outFile.open(destinationPathAndFileName, std::ios::out);
	if (!outFile) {
		std::cerr << "Can't save file: " << destinationPathAndFileName
			  << std::endl;
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

	updateLaplacianHeatMap();

	updatePointSphereDrawingVector();
}

void Mesh::handle_meshContraction(bool automatic) {
	if (m_M.has_garbage()) m_M.collect_garbage();

	if (automatic)
		MC.contractMesh();
	else
		MC.executeContractionStep();
	m_M = MC.getContractedMesh();

	DrawableMesh::updateDrawingVertices();
	updateMeshBuffers();

	updateLaplacianHeatMap();

	updatePointSphereDrawingVector();
}

void Mesh::handle_segmentContraction(bool automatic) {
	alphaValue = 0.4;
	if (automatic)
		SMC.contractMesh();
	else
		SMC.executeContractionStep();
	CGALSurfaceMesh contractedSegment = SMC.getContractedMesh();
	segment.setM(contractedSegment);
	m_showContractedSegment = true;
}

Skeleton Mesh::convertToSkeleton(
    std::vector<std::vector<size_t>> skeletonEdgesInMeshIndices,
    const CGALSurfaceMesh &M) const {
	Skeleton s;
	s.initialize();
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
		//    CGALSurfaceMesh::Point(1 + indexInMesh, 2 + indexInMesh,
		//    0));
		skeletonNodePositions.push_back(
		    M.point(CGALSurfaceMesh::vertex_index(indexInMesh)));
		meshIndices_to_skeletonIndices.insert(
		    std::make_pair(indexInMesh, indexInSkeleton));
		indexInSkeleton++;
	}

	// skeletonEdges with mesh indices -> skeletonEdges with skeleton
	// indices

	std::vector<std::vector<size_t>> skeletonEdgesInSkeletonIndices;
	for (auto edge : skeletonEdgesInMeshIndices) {
		skeletonEdgesInSkeletonIndices.push_back(std::vector<size_t>{
		    meshIndices_to_skeletonIndices[edge[0]],
		    meshIndices_to_skeletonIndices[edge[1]]});
	}

	s.append(skeletonEdgesInSkeletonIndices, skeletonNodePositions, m_PS);
	return s;
}

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
				m_segmentGraph.add_edge(fSegmentIndex,
							neigFSegmentIndex);
				// assert(successfullyAdded);
			}
		}
	}
	std::cout << "Segment graph has:" << m_segmentGraph.num_vertices()
		  << " nodes" << std::endl;
	std::cout << "Segment graph has:" << m_segmentGraph.num_edges()
		  << " edges" << std::endl;
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

void Mesh::updateLaplacianHeatMap() {
	m_laplacianHeatMap.clear();
	std::vector<double> laplacianValues = MC.getLaplacianValues();

	double maxLaplacianValue =
	    *(std::max_element(laplacianValues.begin(), laplacianValues.end()));
	glm::vec3 red(1, 0, 0);

	glm::vec3 green(0, 1, 0);

	double meanLaplacianValue =
	    std::accumulate(laplacianValues.begin(), laplacianValues.end(), 0) /
	    laplacianValues.size();
	glm::vec3 yellow(0.5, 0.5, 0);

	glm::vec3 blue(0, 0, 1);

	for (CGALSurfaceMesh::vertex_index vIndex : m_M.vertices()) {
		PointSphere ps = m_PS;
		ps.setPosition(
		    m_M.point(CGALSurfaceMesh::vertex_index(vIndex)));
		glm::vec3 color;
		if (laplacianValues[size_t(vIndex)] < 0) {
			color = blue;
		} else if (laplacianValues[size_t(vIndex)] <=
			   meanLaplacianValue) {
			float howCloseToYellow =
			    laplacianValues[size_t(vIndex)] /
			    meanLaplacianValue;
			color = glm::normalize((1 - howCloseToYellow) * green +
					       howCloseToYellow * yellow);
		} else {
			float howCloseToRed =
			    laplacianValues[size_t(vIndex)] / maxLaplacianValue;
			color = glm::normalize((1 - howCloseToRed) * yellow +
					       howCloseToRed * red);
		}
		ps.setColor(color);
		m_laplacianHeatMap.push_back(ps);
	}
}
void Mesh::handle_meshConnectivitySurgery() {
	ConnectivitySurgeon CS(m_M);
	CS.execute_connectivitySurgery();
	auto skeletonEdgesInMeshIndices = CS.getSkeletonEdges();
	// m_skeletonMeshMapping = CS.getSkeletonMeshMapping();

	m_skeleton.addSkeletonPart(
	    convertToSkeleton(skeletonEdgesInMeshIndices, m_M));
}

void Mesh::handle_meshRefinementEmbedding() {
	std::cout << "Embedding Refinement.." << std::endl;

	Kernel::Vector_3 node(0, 0, 0);
	std::vector<Kernel::Vector_3> points{Kernel::Vector_3(0, 0, 0),
					     Kernel::Vector_3(0.1, 0.1, 0.1)};

	std::cout << "Node centerness:" << computeNodeCenterness(node, points)
		  << std::endl;

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

void Mesh::handle_segmentConnectivitySurgery() {
	assert(selectedSegmentIndex);
	CGALSurfaceMesh contractedSegment = segment.M();
	ConnectivitySurgeon CS(contractedSegment);
	CS.execute_connectivitySurgery();
	auto skeletonEdgesInMeshIndices = CS.getSkeletonEdges();

	Skeleton s =
	    convertToSkeleton(skeletonEdgesInMeshIndices, contractedSegment);
	m_skeleton.addSkeletonPart(s, selectedSegmentIndex.get());
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
	//    auto
	//    p(contractedSegment.point(CGALSurfaceMesh::vertex_index(vIndex)));
	//    tempPS.setPosition(p);
	//    tempPS.setColor(glm::vec3(1, 0, 0));
	//    tempPS.doubleSize();
	//    m_skeletonNodes.push_back(tempPS);
	//}
	// m_showContractedSegment = false;
}

void Mesh::updateDrawingVertices(const MeshSegment &copyFrom) {
	size_t index = 0;
	for (auto v :
	     copyFrom.M().vertices()) {  // update drawing vertices in model
		CGALSurfaceMesh::Point p = copyFrom.M().point(v);
		m_vertices[copyFrom.vertexCorrespondence[index]].Position =
		    glm::vec3(p.x(), p.y(), p.z());
		index++;
	}
}
