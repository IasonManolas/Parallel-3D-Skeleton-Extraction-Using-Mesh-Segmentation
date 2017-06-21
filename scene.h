#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include <QVector2D>
#include <QVector3D>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Ray_3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "axes.h"
#include "camera.h"
#include "cgaltypedefs.h"
#include "directionallight.h"
#include "mypolyhedron.h"
#include "pointsphere.h"
#include "ray_cast_picking.h"
#include "shader.h"

class Scene {
public:
  float scaleFactor;
  Camera camera{glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f)};
  DirectionalLight light{glm::vec3(1.0f), camera.getPosition(),
                         glm::normalize(-camera.getPosition())};
  MyPolyhedron P;
  PointSphere intersectionSphere;
  bool showIntersection{false};

  Scene() {}

  void Draw(Shader *modelShader, Shader *axisShader) {
    setMeshUniforms(modelShader, axisShader);
    modelShader->Use();
    P.Draw();
    if (showAxes) {
      axisShader->Use();
      sceneAxes.Draw();
    }
    //	if (showIntersection) {
    //	    setIntersectionSphereUniforms(modelShader);
    //	    intersectionSphere.Draw();
    //	}
  }
  void updateProjectionMatrix(int w, int h) {
    projectionMatrix =
        glm::perspective(camera.fov, float(w) / float(h), nearPlane, farPlane);
  }

  void loadMesh(std::string filename) {
    camera.resetCamera();
    light.changeLightDirection(camera.getPosition());

    P = MyPolyhedron{};
    P.load(filename);
  }
  void setShowAxes(bool value) { showAxes = value; }

  boost::optional<std::size_t> getIntersectingSegmentIndex(const int &mouseX,
                                                           const int &mouseY,
                                                           const int width,
                                                           const int height) {
    Ray_intersection intersection;
    bool intersectionFound =
        rayIntersectsPolyhedron(mouseX, mouseY, width, height, intersection);
    if (intersectionFound) {
      CGALSurfaceMesh::Face_index intersectingTriangleIndex;
      intersectingTriangleIndex = intersection->second;
      return P.getSegmentIndex(intersectingTriangleIndex);
      // Point *p = boost::get<Point>(&(intersection->first));
      // intersectionIndex = findClosestVertex(
      //    *p, intersectingTriangleIndex); // returns vertices[someIndex]
      // glm::vec3 intersectionPos = glm::vec3(
      //    modelMatrix * glm::vec4(vertices[intersectionIndex].Position, 1.0));
      // intersectionPosition =
      //    Point(intersectionPos.x, intersectionPos.y, intersectionPos.z);
    } else {
      return boost::none;
    }
  }

  bool rayIntersectsPolyhedron(const int &mouseX, const int &mouseY,
                               const int width, const int height,
                               Ray_intersection &intersection) {
    glm::vec3 camPos = camera.getPosition();
    float x = 2.0 * mouseX / width - 1;
    float y = 1 - (2.0 * mouseY) / height;
    float z = -1; // we want the ray to point into the screen
    glm::vec3 ndcs(x, y, z);
    glm::vec4 ray_clip(ndcs.x, ndcs.y, ndcs.z, 1.0);

    glm::vec4 ray_eye = glm::inverse(projectionMatrix) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
    glm::vec4 ray_wor4 = glm::inverse(camera.getViewMat()) * ray_eye;

    glm::vec3 ray_wor(ray_wor4.x, ray_wor4.y, ray_wor4.z);
    ray_wor = glm::normalize(ray_wor);

    CGAL::Ray_3<Kernel> ray(
        Kernel::Point_3(camPos.x, camPos.y, camPos.z),
        Kernel::Direction_3(ray_wor.x, ray_wor.y, ray_wor.z));

    CGAL::Point_3<Kernel> intersectionPosition;
    intersection = P.intersects(ray);
    if (intersection) {

      //      intersectionSphere.setIntersectingVertexIndex(vertexIndex);
      //      intersectionSphere.setPosition(intersectionPosition);
      // for (int i = 0; i < P.vertices.size(); ++i) {
      //    P.vertices[i].Color = glm::vec3(0, 0, 0);
      //    if (P.vertices[i].Position == P.vertices[vertexIndex].Position) {
      //        P.vertices[i].Color = glm::vec3(1, 0, 0);
      //        std::cout << "intersecting index is:" << i << std::endl;
      //    }
      //}
      return true;
    }
    return false;
    // showIntersection = intersectionFound;
  }

  void processMouseMovement(const QVector2D &mouseDV) {
    camera.processMouseMovement(mouseDV);
    light.changeLightDirection(camera.getPosition());
  }

private:
  //    DirectionalLight
  //    light{glm::vec3(1.0f),camera.getPosition(),glm::normalize(-camera.getPosition())};
  Axes sceneAxes{};
  bool showAxes{false};
  glm::mat4 projectionMatrix{glm::mat4(1.0f)};
  float nearPlane{0.1};
  float farPlane{100};

  void setProjectionMatrixUniform(Shader *shader) {
    glUniformMatrix4fv(glGetUniformLocation(shader->programID, "projection"), 1,
                       GL_FALSE, glm::value_ptr(projectionMatrix));
  }
  void setMeshUniforms(Shader *modelShader, Shader *axisShader) {
    modelShader->Use();
    light.setUniforms(modelShader);
    P.setUniforms(modelShader);
    camera.setUniforms(modelShader);
    setProjectionMatrixUniform(modelShader);

    axisShader->Use();
    setProjectionMatrixUniform(axisShader);
    glUniformMatrix4fv(glGetUniformLocation(axisShader->programID, "model"), 1,
                       GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
    camera.setUniforms(axisShader);
  }
  void setIntersectionSphereUniforms(Shader *shader) {
    shader->Use();
    intersectionSphere.setUniforms(shader);
  }
};

#endif // SCENE_H
