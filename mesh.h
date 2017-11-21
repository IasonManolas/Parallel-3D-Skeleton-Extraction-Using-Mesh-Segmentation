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
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
// sqrt
#include <CGAL/Point_3.h>
#include <CGAL/number_utils.h>
#include <CGAL/squared_distance_3.h>
// getMeshSegment
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/aff_transformation_tags.h>

//#include <CGAL/boost/graph/helpers.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>

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
#include "meshskeleton.h"
#include "pointsphere.h"
#include "shader.h"
#include "undirectedgraph.h"
#include "refinementembedding.h"

class Mesh : public DrawableMesh {

public:
  Mesh()
      : /*skeleton(Skeleton(m_PS, m_modelMatrix)),*/ m_skeleton(
            m_segmentGraph) {}

  void load(std::string filename);
  void setUniforms(Shader *shader) override;
  std::size_t getCorrespondingSegmentIndex(
      const CGALSurfaceMesh::Face_index face_index) const;
  Ray_intersection intersects(Kernel::Ray_3 ray) const;
  MeshSegment getMeshSegment() const;
  void handle_segmentSelection(Ray_intersection);
  void inflationDeflationDeformer(float deformationFactor); // TODO private
  size_t constructSegmentMap();                             // TODO private
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
  void handle_drawing(Shader *modelShader, Shader *edgeShader);
  void handle_saveModel(std::string destinationPathAndFileName);
  void handle_saveSegment(std::string destinationPathAndFileName);
  void setPointSphere(PointSphere);
  void handle_showVerticesStateChange(int state);
  void handle_clearSkeleton() { m_skeleton.clear(); }
  void handle_segmentSkeletonization();
  void handle_skeletonization();
  void handle_laplacianHeatMapStateChange(bool state);

  // std::vector<size_t> getVertexIndicesWithHighLaplacianValue();
  // public data members
public:
  bool segmentsComputed{false};
  // Shader *modelShader;
  //    CGALPolyhedron P;
  MeshContractor MC;
  //    std::vector<Kernel::Vector_3> normals;

  // Skeleton skeleton{Skeleton(m_PS, m_modelMatrix)};
  // private member functions

private:
  // void setIntersectingTriangleUniform(int faceIndex);
  int findClosestVertex(Point intersectionPoint,
                        CGALSurfaceMesh::Face_index intersectingFaceIndex);
  void normalizeMeshViaModelMatrix();
  void inflation_handler();
  void deflation_handler();
  void updateDrawingVertices(const MeshSegment &copyFrom);
  void resetMeshAttributes();
  void populateVerticesAndIndices(std::string filename);
  void drawThisMesh(Shader *);
  Skeleton
  convertToSkeleton(std::vector<std::vector<size_t>> skeletonEdgesInMeshIndices,
                    const CGALSurfaceMesh &) const;
  void
  constructSegmentGraph(size_t numberOfSegments); // populates m_segmentGraph
  void updatePointSphereDrawingVector();
  void updateLaplacianHeatMap();

  // CGALSurfaceMesh::Point computeCenterOfMass(
  //    std::vector<size_t> vertexIndices); // TODO merge to meshMeasuring
  // private data members
private:
  PointSphere m_PS;
  MeshSkeleton m_skeleton;
  Facet_int_map m_segmentFaceMap;
  UndirectedGraph m_segmentGraph;
  boost::optional<size_t> selectedSegmentIndex{boost::none};

  MeshContractor SMC; // segment mesh contractor
  MeshSegment segment{MeshSegment(m_modelMatrix)};
  bool m_showContractedSegment{false};
  // std::vector<std::vector<size_t>> m_skeletonMeshMapping; // used in
  // Refinement
  std::vector<PointSphere> pointSphereDrawingVector; // holds all the points
                                                     // that should be drawn on
                                                     // the model
  std::vector<PointSphere> m_laplacianHeatMap;
  bool m_showPointSpheresOnVertices{false};
  bool m_showLaplacianHeatMap{false};
  // Embedding. NOTE
  // should not be
  // present in the
  // final version
};

#endif // MYPOLYHEDRON_H
