#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

//#include <unordered_set>
#include <algorithm>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
// segmentation
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
// ray shooting
// sqrt
#include <CGAL/Point_3.h>
#include <CGAL/number_utils.h>
#include <CGAL/squared_distance_3.h>
// getMeshSegment
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/aff_transformation_tags.h>

//#include <CGAL/boost/graph/helpers.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/optional/optional_io.hpp>
//#include <QOpenGLContext>

#include "meshloader.h"
//#include "meshmeasuring.h"
#include "cgaltypedefs.h"
#include "connectivitysurgeon.h"
#include "drawablemesh.h"
#include "material.h"
#include "meshcontractor.h"
#include "meshsegment.h"
//#include "meshskeleton.h"
#include "pointsphere.h"
#include "refinementembedding.h"
#include "shader.h"
#include "undirectedgraph.h"

class Mesh : public DrawableMesh {
       public:
	Mesh(const PointSphere &ps)
	    : m_PS(ps),
	      /*skeleton(Skeleton(m_PS, m_modelMatrix)),*/ m_skeleton(
		  ps, m_segmentGraph) {}

	void load(std::string filename);
	void setUniforms(Shader *shader) override;
	std::size_t getCorrespondingSegmentIndex(
	    const CGALSurfaceMesh::Face_index face_index) const;
	Ray_intersection intersects(Kernel::Ray_3 ray) const;
	// MeshSegment getMeshSegment() const;
	void handle_segmentSelection(Ray_intersection);
	void inflationDeflationDeformer(
	    float deformationFactor);  // TODO private
	size_t constructSegmentMap();  // TODO private
	void assignSegmentColors();
	size_t computeSegments();
	void colorPickedSegment();
	void handle_showSegments();
	void handle_meshContractionReversing();
	void handle_meshContraction(bool automatic);
	void handle_segmentContraction(bool automatic);
	void handle_meshConnectivitySurgery();
	void handle_segmentConnectivitySurgery();
	void handle_meshRefinementEmbedding();
	void handle_inflation() { inflation_handler(); }
	void handle_deflation() { deflation_handler(); }
	void handle_drawing(Shader *activeModelShader, Shader *edgeShader,
			    Shader *nodeShader);
	void handle_saveModel(std::string destinationPathAndFileName);
	void handle_saveSegment(std::string destinationPathAndFileName);
	// void setPointSphere(PointSphere);
	void handle_showVerticesStateChange(int state);
	void handle_clearSkeleton() {
		m_skeleton.clear();
		// old skeleton:
		// m_skeleton.setNumberOfSegments(m_numberOfSegments);
	}
	void handle_segmentSkeletonization();
	// void handle_skeletonization();

	void skeletonize();
	void runSkeletonizationMethods();
	void parallelSkeletonization();
	void serialSkeletonizationUsingSegments();
	// std::vector<size_t> getVertexIndicesWithHighLaplacianValue();
	// public data members

       private:
	int findClosestVertex(
	    Point intersectionPoint,
	    CGALSurfaceMesh::Face_index intersectingFaceIndex);
	void normalizeMeshViaModelMatrix();
	void segmentMesh();
	void inflation_handler();
	void deflation_handler();
	void updateDrawingVertices(const MeshSegment &copyFrom);
	void resetMeshAttributes();
	void populateVerticesAndIndices(std::string filename);
	void drawThisMesh(Shader *);
	void constructSegmentGraph(
	    size_t numberOfSegments);  // populates m_segmentGraph
	void updatePointSphereDrawingVector();
	CGALSurfaceMesh constructSelectedSegmentSurfaceMesh(unsigned int) const;

	void MySkeletonization(unsigned int segmentIndex);
	void fillMeshHoles();

       private:
	const PointSphere &m_PS;
	Facet_int_map m_segmentFaceMap;
	UndirectedGraph m_segmentGraph;
	boost::optional<size_t> selectedSegmentIndex{boost::none};

	MeshContractor SMC;  // segment mesh contractor
	MeshSegment m_segment{m_modelMatrix, m_segmentFaceMap,
			      glm::vec3(1, 0, 0), m_vertices, m_M};
	bool m_showContractedSegment{false};
	// std::vector<std::vector<size_t>> m_skeletonMeshMapping; // used in
	// Refinement
	std::vector<PointSphere>
	    pointSphereDrawingVector;  // holds all the points
				       // that should be drawn on
				       // the model
	std::vector<PointSphere> m_laplacianHeatMap;
	bool m_showPointSpheresOnVertices{false};
	bool m_showLaplacianHeatMap{false};
	CGALSurfaceMesh m_originalMesh;

	std::string filename;
	size_t m_numberOfSegments{0};
	// Embedding. NOTE
	// should not be
	// present in the
	// final version
       public:
	bool segmentsComputed{false};
	Skeleton m_skeleton{m_PS, m_segmentGraph};
	// Shader *modelShader;
	//    CGALPolyhedron P;
	MeshContractor MC;
	//    std::vector<Kernel::Vector_3> normals;

	// Skeleton skeleton{Skeleton(m_PS, m_modelMatrix)};
	// private member functions
};

#endif  // MYPOLYHEDRON_H
