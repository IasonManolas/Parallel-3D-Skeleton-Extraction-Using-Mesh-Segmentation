#ifndef MYPOLYHEDRON_H
#define MYPOLYHEDRON_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
// segmentation
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
// ray shooting
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
//#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
// sqrt
#include <CGAL/Point_3.h>
#include <CGAL/number_utils.h>
#include <CGAL/squared_distance_3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <boost/optional/optional_io.hpp>
//#include <QOpenGLContext>
struct MyVertex {
  glm::vec3 Position;
  glm::vec3 Normal;
  glm::vec3 Color{glm::vec3(0, 0, 0)};
};

#include "meshloader.h"
//#include "meshmeasuring.h"
#include "material.h"
#include "shader.h"

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CGALPolyhedron = CGAL::Polyhedron_3<Kernel>;
using CGALSurfaceMesh = CGAL::Surface_mesh<Kernel::Point_3>;
using HalfedgeDS = CGALPolyhedron::HalfedgeDS;
using Ray = Kernel::Ray_3;
using Point = Kernel::Point_3;

class MyPolyhedron {
public:
  double averageEdgeLength{1};
  MyPolyhedron() {}
  void resetMeshAttributes() {
    indices.clear();
    vertices.clear();
    centerOfMass = glm::vec3(1.0);
    modelMatrix = glm::mat4(1.0);
  }
  void load(std::string filename) {
    resetMeshAttributes();
    std::tie(indices, vertices) =
        meshLoader::load(filename); // vertices contains coords & normals
    setupDrawingBuffers();

    // Find the model matrix that normalizes the mesh
    centerOfMass = meshMeasuring::findCenterOfMass(vertices);
    maxDim = meshMeasuring::findMaxDimension(vertices);
    normalizeMeshViaModelMatrix();
    buildPolygonMesh();
    averageEdgeLength = meshMeasuring::findAverageEdgeLength(M);

    printMeshInformation();
  }

  void computeOneRingNeighbours(int vertexIndex){
	std::cout<<"Computing neighbours.."<<std::endl;
	std::cout<<"intersecting index is:"<<vertexIndex<<std::endl;

  }
  void printDebugInformation() const {
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
  void setUniforms(Shader *shader) {
    modelShader = shader; // maybe this is not good design?
    material.setUniforms(modelShader);
    glUniformMatrix4fv(glGetUniformLocation(modelShader->programID, "model"), 1,
                       GL_FALSE, glm::value_ptr(modelMatrix));
    setIntersectingTriangleUniform((size_t)intersectingTriangleIndex);
  }
  void setIntersectingTriangleUniform(int faceIndex) {
    int location =
        glGetUniformLocation(modelShader->programID, "intersectingFace");
    glUniform1i(location, int(faceIndex));
  }
  void Draw() {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

  bool intersects(Kernel::Ray_3 ray,
                  int& intersectionIndex,CGAL::Point_3<Kernel> &intersectionPosition) {

    typedef CGAL::AABB_face_graph_triangle_primitive<CGALSurfaceMesh> Primitive;
    typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
    typedef CGAL::AABB_tree<Traits> Tree;
    Tree tree(faces(M).first, faces(M).second, M);
    typedef boost::optional<
        Tree::Intersection_and_primitive_id<Kernel::Ray_3>::Type>
        Ray_intersection;
    boost::optional<int> a = 3;

    std::cout << "Computing intersection" << std::endl;
    Ray_intersection intersection = tree.first_intersection(ray);
    std::cout << "Computed intersection" << std::endl;
    if (intersection) {
      std::cout << "Intersection(s) found!" << std::endl;
      if (boost::get<Point>(&(intersection->first))) {
        Point *p = boost::get<Point>(&(intersection->first));
        intersectingTriangleIndex = intersection->second;
        intersectionIndex= findClosestVertex(
            *p, intersectingTriangleIndex); // returns vertices[someIndex]
        glm::vec3 intersectionPos =
            modelMatrix * glm::vec4(vertices[intersectionIndex].Position, 1.0);
        intersectionPosition =
            Point(intersectionPos.x, intersectionPos.y, intersectionPos.z);
        return true;
      }
    }
    std::cout << "No Intersection found." << std::endl;
    return false;
  }

protected:
  Shader *modelShader;
  CGALSurfaceMesh::Face_index intersectingTriangleIndex{0};
  CGALPolyhedron P;
  CGALSurfaceMesh M;
  std::vector<MyVertex> vertices;
  std::vector<GLuint> indices;
  //    std::vector<Kernel::Vector_3> normals;
  GLuint VAO, VBO, EBO;
  glm::vec3 centerOfMass{0, 0, 0};
  float maxDim{1.0};
  glm::mat4 modelMatrix{1.0};
  Material material{Material(glm::vec3(0, 0, 0), glm::vec3(0.5, 0.5, 0),
                             glm::vec3(0.6, 0.6, 0.5), 128 * 0.25)};

  int findClosestVertex(Point intersectionPoint,
                        CGALSurfaceMesh::Face_index intersectingFaceIndex) {
    CGALSurfaceMesh::halfedge_index beginHalfedge =
        M.halfedge(intersectingFaceIndex);
    CGALSurfaceMesh::halfedge_index h = beginHalfedge;
    CGALSurfaceMesh::vertex_index vIndex = M.source(h);
    Kernel::Point_3 vertex = M.point(vIndex);
    double minDistance = CGAL::squared_distance(vertex, intersectionPoint);
    int closestVertexIndex = (int)vIndex;
    //       std::cout << "vertices around vertex " <<v<< std::endl;
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

  void buildPolygonMesh() {
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
    //       CGAL::Polygon_mesh_processing::orient_polygon_soup(points,polygons);
    std::cout << "is polygon soup polygon mesh:"
              << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
                     polygons)
              << std::endl;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,
                                                                polygons, M);
    //       if (CGAL::is_closed(M) &&
    //       (!CGAL::Polygon_mesh_processing::is_outward_oriented(M)))
    //         CGAL::Polygon_mesh_processing::reverse_face_orientations(M);

    //        std::map<vd,Kernel::Vector_3> normalsMap;
    //        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normalsMap));
    std::cout << "Finished building Polygon Mesh." << std::endl;
  }
  void buildPolyhedron() {
    P.clear();
    std::cout << "Building Polyhedron.." << std::endl;
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
    CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
    std::cout << "is polygon soup polygon mesh:"
              << CGAL::Polygon_mesh_processing::is_polygon_soup_a_polygon_mesh(
                     polygons)
              << std::endl;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points,
                                                                polygons, P);
    if (CGAL::is_closed(P) &&
        (!CGAL::Polygon_mesh_processing::is_outward_oriented(P)))
      CGAL::Polygon_mesh_processing::reverse_face_orientations(P);

    //        std::map<vd,Kernel::Vector_3> normalsMap;
    //        CGAL::Polygon_mesh_processing::compute_vertex_normals(P,boost::make_assoc_property_map(normalsMap));
    std::cout << "Finished building Polyhedron." << std::endl;
  }
  void normalizeMeshViaModelMatrix() {
    float scaleFactor = 1.0 / maxDim;
    modelMatrix = glm::scale(modelMatrix, glm::vec3(scaleFactor));

    glm::vec3 translationVector(-centerOfMass);
    modelMatrix = glm::translate(modelMatrix, translationVector);
  }

  glm::mat4 getModelMatrix() const { return modelMatrix; }

  void printMeshInformation() const {
    std::cout << "Number of vertices:" << vertices.size() << std::endl;
    std::cout << "Number of faces:" << indices.size() / 3 << std::endl;
  }

  void setupDrawingBuffers() {
    std::cout << "Entering setupDrawingBuffers().." << std::endl;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(MyVertex),
                 &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
                 &indices[0], GL_STATIC_DRAW);

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
    //   std::cout<<"printDebugInformation was called in
    //   :"<<__func__<<std::endl;
    //        printDebugInformation();

    std::cout << "Exiting setupDrawingBuffers().." << std::endl;
  }
};

#endif // MYPOLYHEDRON_H
