#include "meshcontractor.h"
#include "debug_meshcontractor.h"
//#include <Eigen/unsupported/Eigen/SparseExtra>
double MeshContractor::m_volumeThreshold = std::pow(10, -4);
double MeshContractor::m_areaThreshold = std::pow(10, -4);
SpMatrix concatenateVertically(SpMatrix A, SpMatrix B) {
	// A
	//---
	// B
	assert(A.cols() == B.cols());
	std::vector<EigenTriplet> tripletVector;
	for (int k = 0; k < A.outerSize(); ++k) {
		for (SpMatrix::InnerIterator it(A, k); it; ++it) {
			tripletVector.push_back(
			    EigenTriplet(it.row(), it.col(), it.value()));
		}
		for (SpMatrix::InnerIterator it(B, k); it; ++it) {
			tripletVector.push_back(EigenTriplet(
			    it.row() + A.rows(), it.col(), it.value()));
		}
	}
	SpMatrix M(A.rows() + B.rows(), A.cols());
	M.setFromTriplets(tripletVector.begin(), tripletVector.end());

	return M;
}

MeshContractor::MeshContractor(CGALSurfaceMesh meshToContract /*,
                               std::vector<uint> indices*/)
    : m_M(meshToContract) {
	// Eigen::initParallel();

	// Eigen::setNbThreads(6);
	// int nthreads = Eigen::nbThreads();
	// std::cout << "THREADS = " << nthreads << std::endl; // returns '1'

	if (!CGAL::is_closed(m_M)) {
		using Halfedge_handle =
		    boost::graph_traits<CGALSurfaceMesh>::halfedge_descriptor;
		using Facet_handle =
		    boost::graph_traits<CGALSurfaceMesh>::face_descriptor;
		BOOST_FOREACH (Halfedge_handle h, halfedges(m_M)) {
			if (m_M.is_border(h)) {
				std::vector<Facet_handle> patch_facets;
				CGAL::Polygon_mesh_processing::triangulate_hole(
				    m_M, h, std::back_inserter(patch_facets));
			}
		}
	}
	m_originalVolume = CGAL::Polygon_mesh_processing::volume(m_M);
	m_originalArea = CGAL::Polygon_mesh_processing::area(m_M);
	m_Wh = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
	// m_Wh = DiagMatrix(m_M.number_of_vertices());
	m_Wh.setIdentity();
	m_Wh *= m_Wh0;

	double averageFaceArea =
	    CGAL::Polygon_mesh_processing::area(m_M) / (m_M.number_of_faces());
	m_Wl = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());
	// m_Wl = DiagMatrix(m_M.number_of_vertices());
	m_Wl.setIdentity();
	double WlInitialWeight = 0.001 * std::sqrt(averageFaceArea);
	m_Wl *= WlInitialWeight;
	// computeLaplaceOperatorUsingCGAL(); //?

	// uint numRowsF = indices.size() / 3;
	// F.resize(numRowsF, 3);
	// for (int i = 0; i < indices.size(); i += 3) {
	//  uint faceIndex = i / 3;
	//  F(faceIndex, 0) = indices[i];
	//  F(faceIndex, 1) = indices[i + 1];
	//  F(faceIndex, 2) = indices[i + 2];
	//}
	isVertexFixed.resize(m_M.number_of_vertices(), false);
	std::cout << "Initializing laplace operator.." << std::endl;
	computeLaplaceOperator();
	hasNaN(m_L);
	hasInfinity(m_L);
	m_A = Vector::Zero(m_M.number_of_vertices());
	computeOneRingAreaVector();
	m_A0 = m_A;
	m_previousVertexPositions.push_back(constructVertexMatrix());
	m_previousDeltaCoords = EigenMatrix::Zero(m_M.number_of_vertices(), 3);
	//
	for (auto fi : m_M.faces())
		m_initialFaceAreas.push_back(
		    CGAL::Polygon_mesh_processing::face_area(fi, m_M));
}

void MeshContractor::contractMesh() {
	while (CGAL::Polygon_mesh_processing::volume(m_M) / m_originalVolume >
		   m_volumeThreshold &&
	       m_iterationsCompleted < m_maxNumOfIterations &&
	       !m_degenerateFaceIsPresent &&
	       CGAL::Polygon_mesh_processing::area(m_M) / m_originalArea >
		   m_areaThreshold) {
		executeContractionStep();
	}
}

// void MeshContractor::printFixedVertices(EigenMatrix verticesMatrix) {
//  std::vector<size_t> fixedVerticesIndices = getFixedVertices();
//  EigenMatrix fixedVerticesPositions(fixedVerticesIndices.size(), 3);
//  for (size_t i = 0; i < fixedVerticesPositions.rows(); i++) {
//    fixedVerticesPositions(i, 0) = verticesMatrix(fixedVerticesIndices[i], 0);
//    fixedVerticesPositions(i, 1) = verticesMatrix(fixedVerticesIndices[i], 1);
//    fixedVerticesPositions(i, 2) = verticesMatrix(fixedVerticesIndices[i], 2);
//  }
//  //std::cout << "Fixed vertices positions are:" << std::endl;
//  //std::cout << fixedVerticesPositions << std::endl;
//}

// void MeshContractor::computeFixedVertices() {
//  double faceRatioThreshold = 100000;
//  fixedVertices.clear();
//  for (CGALSurfaceMesh::face_index fi : m_M.faces()) {
//    if (m_initialFaceAreas[size_t(fi)] /
//            CGAL::Polygon_mesh_processing::face_area(fi, m_M) >
//        faceRatioThreshold) {
//      BOOST_FOREACH (CGALSurfaceMesh::vertex_index vd,
//                     vertices_around_face(m_M.halfedge(fi), m_M)) {
//        std::cout << vd << std::endl;
//        // isVertexFixed[size_t(vd)]=true;
//        fixedVertices.insert(size_t(vd));
//      }
//    }
//  }
//}
void setMaximumNumber(SpMatrix &M, double maximumNumber) {
	for (int k = 0; k < M.outerSize(); ++k)
		for (SpMatrix::InnerIterator it(M, k); it; ++it) {
			if (it.value() > maximumNumber) {
				std::cout << "Changed from:" << it.value()
					  << std::endl;
				M.coeffRef(it.row(), it.col()) = maximumNumber;
				std::cout << "to:" << it.value() << std::endl;
			}
		}
}

void MeshContractor::executeContractionReversingStep() {
	std::cout << "Contracting Mesh.." << std::endl;

	if (m_iterationsCompleted > 0) {
		m_iterationsCompleted--;
		EigenMatrix V =
		    m_previousVertexPositions[m_iterationsCompleted];
		// printFixedVertices(V);
		// printMatrix(V, "NewV");
		updateMeshPositions(V);
		// computeFixedVertices();

		// printSparseMatrix(m_Wl, "Wl");
		m_Wl /= m_Sl;
		// printVector(m_A, "m_A");
		computeOneRingAreaVector();
		// printSparseMatrix(m_Wh, "Wh");
		updateWh();
		// std::cout<<"number of fixed
		// vertices:"<<fixedVertices.size()<<std::endl;
		// printDiagonalElementsOfSparseMatrix(m_L, "L");
		// m_L = computeLaplaceOperatorUsingIGL();
		// m_previousDeltaCoords = m_L * V;
		computeLaplaceOperator();
		VertexWithAttribute maxLVertex =
		    getMaximumAbsoluteDiagonalElement(m_L);
		std::cout << "Maximum L(" << maxLVertex.index
			  << ")=" << maxLVertex.value << std::endl;
		std::cout << "maxL*Wl=" << maxLVertex.value << "*"
			  << m_Wl.coeff(0, 0) << "="
			  << maxLVertex.value * m_Wl.coeff(0, 0) << std::endl;
		std::cout << "Wh(" << maxLVertex.index << ")="
			  << m_Wh.coeff(maxLVertex.index, maxLVertex.index)
			  << std::endl;
		//  setMaximumNumber(m_L,maxNumber);
		// std::cout<<"Maximum element of L
		// is:"<<getMaximumAbsoluteDiagonalElement(m_L)<<std::endl;
	}
	std::cout << "Number of iterations completed:" << m_iterationsCompleted
		  << std::endl;
	std::cout << "current volume/original volume="
		  << CGAL::Polygon_mesh_processing::volume(m_M) /
			 m_originalVolume
		  << std::endl;
}

void MeshContractor::setVolumeThreshold(double volumeThreshold) {
	MeshContractor::m_volumeThreshold = volumeThreshold;
}

std::vector<double> MeshContractor::getLaplacianValues() {
	std::vector<double> laplacianValues(m_M.number_of_vertices());

	for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
		laplacianValues[i] = -m_L.coeff(i, i);
	}
	return laplacianValues;
}

void MeshContractor::executeContractionStep() {
	std::cout << "Contracting Mesh.." << std::endl;
	EigenMatrix V = constructVertexMatrix();
	// printMatrix(V, "V");
	// printFixedVertices(V);

	V = solveForNewVertexPositions(V);
	if (m_iterationsCompleted + 1 == m_previousVertexPositions.size())
		m_previousVertexPositions.push_back(V);
	// printFixedVertices(V);
	// printMatrix(V, "NewV");
	updateMeshPositions(V);
	// computeFixedVertices();
	// printSparseMatrix(m_Wl, "Wl");
	updateWl();
	// printVector(m_A, "m_A");
	computeOneRingAreaVector();
	// printSparseMatrix(m_Wh, "Wh");
	updateWh();
	// std::cout<<"number of fixed
	// vertices:"<<fixedVertices.size()<<std::endl;
	// printDiagonalElementsOfSparseMatrix(m_L, "L");
	// m_L = computeLaplaceOperatorUsingIGL();
	// m_previousDeltaCoords = m_L * V;
	computeLaplaceOperator();
	// printDiagonalElementsOfSparseMatrix(m_L, "L");
	if (maxLtoWhIndex != -1) previousMaxLtoWhIndex = maxLtoWhIndex;
	VertexWithAttribute maxLtoWhVertex = getMaximumLtoWhRatio(m_L, m_Wh);
	std::cout << "Vertex " << maxLtoWhVertex.index
		  << " has the maximum ratio:r=|L/Wh|="
		  << "|"
		  << m_L.coeff(maxLtoWhVertex.index, maxLtoWhVertex.index)
		  << "/"
		  << m_Wh.coeff(maxLtoWhVertex.index, maxLtoWhVertex.index)
		  << "| =" << maxLtoWhVertex.value << std::endl;
	maxLtoWhIndex = maxLtoWhVertex.index;

	VertexWithAttribute maxLVertex = getMaximumAbsoluteDiagonalElement(m_L);
	std::cout << "Maximum L(" << maxLVertex.index
		  << ")=" << maxLVertex.value << std::endl;
	std::cout << "maxL*Wl=" << maxLVertex.value << "*" << m_Wl.coeff(0, 0)
		  << "=" << maxLVertex.value * m_Wl.coeff(0, 0) << std::endl;
	std::cout << "Wh(" << maxLVertex.index
		  << ")=" << m_Wh.coeff(maxLVertex.index, maxLVertex.index)
		  << std::endl;

	size_t numPositiveDiagonalElements =
	    getNumberOfPositiveDiagonalElements(m_L);
	std::cout << "Number of positive diagonal elements:"
		  << numPositiveDiagonalElements << std::endl;
	//  setMaximumNumber(m_L,maxNumber);
	// std::cout<<"Maximum element of L
	// is:"<<getMaximumAbsoluteDiagonalElement(m_L)<<std::endl;

	m_iterationsCompleted++;
	std::cout << "Number of iterations completed:" << m_iterationsCompleted
		  << std::endl;
	std::cout << "current volume/original volume="
		  << CGAL::Polygon_mesh_processing::volume(m_M) /
			 m_originalVolume
		  << std::endl;
	std::cout << "current area/original area="
		  << CGAL::Polygon_mesh_processing::area(m_M) / m_originalArea
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
	// std::vector<size_t> fixedIndices = getFixedVertices();
	// std::cout << "Number of fixed vertices:" << fixedIndices.size() <<
	// std::endl;

	// for (size_t index = 0; index < m_M.number_of_vertices(); index++) {
	// for (auto index : fixedIndices) {
	// std::cout << "Vertex with index:" << index << " will not be removed."
	//          << std::endl;
	// m_Wl.coeffRef(index, index) = 1;
	// m_Wh.coeffRef(index, index) = 1;
	//}

	SpMatrix WlL = m_Wl * m_L;

	SpMatrix A = concatenateVertically(WlL, m_Wh);
	// Eigen::JacobiSVD<EigenMatrix> svd(EigenMatrix(A),
	//                                  Eigen::ComputeThinU |
	//                                  Eigen::ComputeThinV);
	// std::cout << "Eigenvalues that are not exaclty 0:" << std::endl
	//          << svd.nonzeroSingularValues() << std::endl;
	// Eigen::saveMarket(A,"../A.mat");
	// SpMatrix A(WlL.rows() + m_Wh.rows(), WlL.cols());
	// A << WlL, m_Wh;
	EigenMatrix Bupper = EigenMatrix::Zero(WlL.rows(), 3);
	// for (size_t vi : fixedIndices) {
	//  // std::cout << "Before:" << Bupper(vi, 0) << std::endl;
	//  Bupper(vi, 0) = m_previousDeltaCoords(vi, 0);
	//  Bupper(vi, 1) = m_previousDeltaCoords(vi, 1);
	//  Bupper(vi, 2) = m_previousDeltaCoords(vi, 2);
	//  // std::cout << "After:" << Bupper(vi, 0) << std::endl;
	//}
	EigenMatrix Blower = (m_Wh * currentVertexPositions);
	EigenMatrix B(Bupper.rows() + Blower.rows(), 3);
	B << Bupper, Blower;

	// EigenMatrix newVertexPositions =svd.solve(B);

	// Solve system using SimplicialLDLT
	Eigen::SimplicialLDLT<SpMatrix> solver;
	SpMatrix At = A.transpose();
	SpMatrix AtA = A.transpose() * A;
	solver.compute(AtA);
	if (solver.info() != Eigen::Success)
		std::cout << "Decomposition failed!" << std::endl;
	EigenMatrix AtB = A.transpose() * B;
	EigenMatrix newVertexPositions = solver.solve(AtB);
	if (solver.info() != Eigen::Success)
		std::cout << "Solving failed!" << std::endl;
	assert(!hasInfinity(At));
	assert(!hasNaN(At));
	assert(!hasNaN(AtA));
	assert(!hasNaN(AtB));
	assert(!hasInfinity(AtA));
	assert(!hasInfinity(AtB));

	// Eigen::LeastSquaresConjugateGradient<SpMatrix> solver;
	// solver.compute(A);
	// if(solver.info()!=Eigen::Success)
	//    std::cout<<"Decomposition failed!"<<std::endl;
	// EigenMatrix newVertexPositions=solver.solve(B);
	// if(solver.info()!=Eigen::Success)
	//    std::cout<<"Solving failed!"<<std::endl;

	// BiCGSTAB
	// Eigen::BiCGSTAB<SpMatrix> solver;
	// solver.compute(A);
	// EigenMatrix newVertexPositions=solver.solve(B);
	// std::cout << "estimated error: " << solver.error()<< std::endl;

	// Eigen::ConjugateGradient<SpMatrix,Eigen::Lower|Eigen::Upper> solver;
	// solver.compute(A);
	// EigenMatrix newVertexPositions=solver.solve(B);
	// std::cout << "estimated error: " << solver.error()<< std::endl;

	assert(!hasInfinity(m_L));
	assert(!hasInfinity(m_Wl));
	assert(!hasInfinity(WlL));
	assert(!hasNaN(WlL));
	assert(!hasInfinity(B));
	assert(!hasNaN(B));
	assert(!hasNaN(A));
	assert(!hasInfinity(A));
	assert(!hasNaN(newVertexPositions));
	assert(!hasInfinity(newVertexPositions));

	return newVertexPositions;
}

void MeshContractor::updateWl() { m_Wl *= m_Sl; }

void MeshContractor::updateWh() {
	lowOneRingAreaVertices.clear();
	highOneRingAreaVertices.clear();
	for (size_t i = 0; i < m_M.number_of_vertices(); i++) {
		double initialToCurrentAreaRatio = m_A0(i) / m_A(i);
		if (m_A(i) < 0.000000000001) {
			std::cout << "ZERO AREA in function: " << __func__
				  << std::endl;
			m_degenerateFaceIsPresent = true;
		}
		if (initialToCurrentAreaRatio > 100)
			lowOneRingAreaVertices.insert(i);
		else
			highOneRingAreaVertices.insert(i);

		double m_Whi = std::sqrt(initialToCurrentAreaRatio);
		assert(!std::isinf(m_Whi));
		assert(!std::isnan(m_Whi));
		m_Wh.coeffRef(i, i) = m_Wh0 * m_Whi;
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
	       m_M.target(m_M.halfedge(
		   v)));  // make sure target and not source is being used
}

void MeshContractor::computeLaplaceOperator() {
	previousLaplaceOperator = m_L;
	m_L = SpMatrix(m_M.number_of_vertices(), m_M.number_of_vertices());

	std::vector<EigenTriplet> tripletVector;
	std::vector<double> diagonalElements(m_M.number_of_vertices(), 0);
	// Weight_calculator m_weight_calculator(m_M);
	// SpMatrix igl_L = computeLaplaceOperatorUsingIGL();
	problematicVertices.clear();
	for (auto ei : m_M.edges()) {
		size_t i = size_t(m_M.vertex(ei, 0));
		size_t j = size_t(m_M.vertex(ei, 1));
		boost::optional<double> optionalWeight =
		    computeCotangentWeight(ei);
		double weight;
		if (optionalWeight)
			weight = optionalWeight.get() / 2;
		else {
			weight = std::pow(10, 6);
			// fixedVertices.insert(i);
			// fixedVertices.insert(j);
			// weight = previousLaplaceOperator.coeff(i, j);
			// std::cout << "Using previous cotangent weight for:"
			// << i << "," << j
			//          << " with value:" << weight << std::endl;
			// problematicVertices.push_back(i);
			// problematicVertices.push_back(j);
		}
		assert(!std::isnan(weight));
		assert(!std::isinf(weight));
		diagonalElements[i] -= weight;
		diagonalElements[j] -= weight;
		tripletVector.push_back(EigenTriplet(i, j, weight));
		tripletVector.push_back(EigenTriplet(j, i, weight));
	}

	// populate diagonal elements
	for (size_t i = 0; i < diagonalElements.size(); i++) {
		tripletVector.push_back(
		    EigenTriplet(i, i, diagonalElements[i]));
		assert(!std::isnan(diagonalElements[i]));
	}
	m_L.setFromTriplets(tripletVector.begin(), tripletVector.end());
	// assert(!hasNaN(m_L));
	// assert(!hasInfinity(m_L));
}
boost::optional<double> MeshContractor::computeCotangentWeight(
    CGALSurfaceMesh::edge_index ei) {
	CGALSurfaceMesh::halfedge_index hei = m_M.halfedge(ei, 0);
	return computeCotangentWeight(hei);
}

boost::optional<double> MeshContractor::computeCotangentWeight(
    CGALSurfaceMesh::halfedge_index hei) {
	CGALSurfaceMesh::Point a = m_M.point(m_M.source(hei)),
			       b = m_M.point(m_M.target(hei));
	CGALSurfaceMesh::Point c = m_M.point(m_M.target(m_M.next(hei))),
			       d = m_M.point(
				   m_M.target(m_M.next(m_M.opposite(hei))));

	assert(c != b);
	assert(c != a);
	assert(d != a);
	assert(d != b);
	Kernel::Vector_3 v1 = Kernel::Vector_3(c, b),
			 v2 = Kernel::Vector_3(c, a),
			 v3 = Kernel::Vector_3(d, a),
			 v4 = Kernel::Vector_3(d, b);

	boost::optional<double> cot1 = computeCotangentValue(v1, v2);
	boost::optional<double> cot2 = computeCotangentValue(v3, v4);
	if (!cot1 || !cot2) return boost::none;
	double cotWeight = cot1.get() + cot2.get();

	return cotWeight;
}
boost::optional<double> MeshContractor::computeCotangentValue(
    Kernel::Vector_3 a, Kernel::Vector_3 b) {
	double dot_ab = a * b;
	double dot_aa = a.squared_length();
	double dot_bb = b.squared_length();

	assert(!std::isinf(dot_ab));
	assert(dot_aa != 0 && !std::isinf(dot_aa));
	assert(dot_bb != 0 && !std::isinf(dot_bb));

	double cosine = dot_ab / (CGAL::sqrt(dot_aa) * CGAL::sqrt(dot_bb));
	if (!(cosine >= -1 && cosine <= 1)) {
		std::cout << "cosine value out of bounds:" << cosine
			  << std::endl;
		if (cosine < -1)
			cosine = -1;
		else
			cosine = 1;
	}

	double cosineSquared = cosine * cosine;
	if (!(cosineSquared >= 0 && cosineSquared <= 1)) {
		std::cout << "cosineSquared value out of bounds:"
			  << cosineSquared << std::endl;
		if (cosineSquared < 0)
			cosineSquared = 0;
		else
			cosineSquared = 1;
	}
	double sineSquared = double(1.0) - cosineSquared;
	if (!(sineSquared >= 0 && sineSquared <= 1)) {
		std::cout << "sineSquared value out of bounds:" << sineSquared
			  << std::endl;
		if (sineSquared < 0)
			sineSquared = 0;
		else
			sineSquared = 1;
	}
	// double lb=-0.99999999999;
	// double ub=0.999999999999;
	// cosine = (cosine < lb) ? lb : cosine;
	// cosine = (cosine > ub) ? ub : cosine;
	// cosineSquared = (cosineSquared < lb) ? lb : cosineSquared;
	// cosineSquared = (cosineSquared > ub) ? ub : cosineSquared;
	assert(sineSquared >= 0 && sineSquared <= 1);
	double sine = std::sqrt(sineSquared);
	assert(!std::isnan(sine));
	if (sine == 0) {
		std::cout << "SIN==0" << std::endl;
		m_degenerateFaceIsPresent = true;
		return boost::none;
	}
	assert(sine >= -1 && sine <= 1);
	assert(cosine >= -1 && cosine <= 1);
	double cotWeight = cosine / sine;
	return cotWeight;
}

CGALSurfaceMesh MeshContractor::getContractedMesh() const { return m_M; }
