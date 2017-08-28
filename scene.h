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
#include "mesh.h"
#include "meshcontractor.h"
#include "pointsphere.h"
#include "shader.h"
#include "skeleton.h"

class Scene {
public:
  Scene() {}

  void Draw(Shader *modelShader, Shader *axisShader);
  void updateProjectionMatrix(int w, int h);

private:
  PointSphere PS;
  Camera camera{glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                glm::vec3(0.0f, 1.0f, 0.0f)};
  DirectionalLight light{glm::vec3(1.0f), camera.getPosition(),
                         glm::normalize(-camera.getPosition())};
  Axes sceneAxes{};
  bool showAxes{false};
  glm::mat4 projectionMatrix{glm::mat4(1.0f)};
  float nearPlane{0.1};
  float farPlane{100};
  Skeleton meshSkeleton;
  Mesh P;
  Mesh contractedMesh;
  MeshContractor MC;

  bool m_showPointSpheresOnVertices{false};
  std::vector<PointSphere> m_pointSpheresOnVertices;

public:
  void initializeScene();
  void loadMesh(std::string filename);
  // Signal handlers
  void handle_AxesStateChange(bool value);
  void handle_CameraZoomChange(float delta);
  void handle_CameraReset();
  void handle_SegmentSelection(float mousePosX, float mousePosY,
                               int windowWidth, int windowHeight);
  void handle_ShowSegments();
  void handle_SegmentContraction();
  void handle_MeshContraction();
  void handle_MeshInflation();
  void handle_MeshDeflation();
  void handle_MouseMovement(const QVector2D &mouseDV);
  void handle_meshVerticesStateChange(int state);

private:
  void loadPointSphere();
  void setProjectionMatrixUniform(Shader *shader);
  void setSceneUniforms(Shader *modelShader, Shader *axisShader);
  void contractMesh();
  bool rayIntersectsPolyhedron(const int &mouseX, const int &mouseY,
                               const int width, const int height,
                               Ray_intersection &intersection);
  void updatePointSpheresOnVerticesPositions();
};

#endif // SCENE_H
