#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <unordered_set>

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

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <boost/optional/optional_io.hpp>
//#include <QOpenGLContext>

#include "meshloader.h"
//#include "meshmeasuring.h"
#include "cgaltypedefs.h"
#include "connectivitysurgeon.h"
#include "material.h"
#include "meshcontractor.h"
#include "meshsegment.h"
#include "pointsphere.h"
#include "shader.h"

class Mesh {
public:
  Mesh();

  void load(std::string filename);
  void setUniforms(Shader *shader);
  std::size_t
  getSegmentIndex(const CGALSurfaceMesh::Face_index face_index) const;
  Ray_intersection intersects(Kernel::Ray_3 ray) const;
  glm::mat4 getModelMatrix() const { return modelMatrix; }
  void updateMeshBuffers();
  MeshSegment getMeshSegment(size_t segmentIndex) const;
  void handle_segmentSelection(Ray_intersection);
  void inflationDeflationDeformer(float deformationFactor); // TODO private
  void constructSegmentMap();                               // TODO private
  void assignSegmentColors();
  void computeSegments();
  void colorPickedSegment();
  void handle_showSegments();
  void handle_meshContraction();
  void handle_segmentContraction();
  void handle_inflation() { inflation_handler(); }
  void handle_deflation() {
    deflation_handler();
  } // TODO use this type of handlers.I like them.
  void updateDrawingVertices();
  void handle_drawing(Shader *shader);
  void handle_saveModel(std::string destinationPathAndFileName);

  std::vector<size_t> getVertexIndicesWithHighLaplacianValue();
  // public data members
public:
  bool segmentsComputed{false};
  Shader *modelShader;
  //    CGALPolyhedron P;
  CGALSurfaceMesh M; // In world coordinates
  MeshContractor MC;
  //    std::vector<Kernel::Vector_3> normals;

  // private member functions
  std::vector<MyVertex> getVertices() const;

  void setMaterial(const Material &value);

private:
  void setIntersectingTriangleUniform(int faceIndex);
  int findClosestVertex(Point intersectionPoint,
                        CGALSurfaceMesh::Face_index intersectingFaceIndex);
  void buildSurfaceMesh();
  void normalizeMeshViaModelMatrix();
  void printMeshInformation() const {
    std::cout << "Number of vertices:" << vertices.size() << std::endl;
    std::cout << "Number of faces:" << indices.size() / 3 << std::endl;
  }
  bool segmentHasBeenSelected() const;
  void updateVertices(const MeshSegment &from);
  void inflation_handler();
  void deflation_handler();
  void unselectSegment();
  void updateCGALSurfaceMeshVertices(const MeshSegment &copyFrom);
  void updateDrawingVertices(const MeshSegment &copyFrom);
  void testVerticesConsistency() const;
  void handle_drawSpheresOnVertices(Shader *shader);
  void resetMeshAttributes();
  void setupDrawingBuffers();
  void DrawMesh();
  void loadObj(std::string);
  // private data members
private:
  std::vector<glm::vec3> colorPalette{
      // NOTE this should be static
      glm::vec3(128.0, 0, 0),   glm::vec3(255, 0, 0),
      glm::vec3(255, 200, 220), glm::vec3(170.0, 110, 40),
      glm::vec3(255.0, 150, 0), glm::vec3(255.0, 215.0, 180),
      glm::vec3(128, 128, 0),   glm::vec3(255, 235, 0),
      glm::vec3(255, 250, 200), glm::vec3(190, 255, 0),
      glm::vec3(0, 190, 0),     glm::vec3(170, 255, 195),
      glm::vec3(0, 128, 128),   glm::vec3(100, 255, 255),
      glm::vec3(0, 0, 128),     glm::vec3(67, 133, 255),
      glm::vec3(130, 0, 150),   glm::vec3(230, 190, 255),
      glm::vec3(255, 0, 255),   glm::vec3(128, 128, 128)};
  Material material{glm::vec3(0.05, 0.05, 0.05),
                    glm::vec3(0.5, 0.5, 0), // should be static constexpr
                    glm::vec3(0.6, 0.6, 0.5), 128 * 0.25};

  GLuint VAO, VBO, EBO;
  Facet_int_map segment_property_map;
  double averageEdgeLength{1};
  int selectedSegmentIndex{-1};

  glm::vec3 centerOfMass{0, 0, 0}; // in model coordinates
  float maxDim{1.0};               // in model coordinates
  glm::mat4 modelMatrix{1.0};      //  modelSpace->worldSpace
  std::vector<MyVertex> vertices;  // in model coordinates
  std::vector<uint> indices;
  double alphaValue{1.0f};
  MeshContractor SMC; // segment mesh contractor
  MeshSegment segment;
};

#endif // MYPOLYHEDRON_H
