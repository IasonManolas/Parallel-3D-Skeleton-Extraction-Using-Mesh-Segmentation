#include "scene.h"

void Scene::Draw(Shader *modelShader, Shader *axisShader) {
  setSceneUniforms(modelShader, axisShader);
  modelShader->Use();
  P.setUniforms(modelShader);
  P.Draw();
  if (m_showPointSpheresOnVertices) {
    for (auto ps : m_pointSpheresOnVertices) {
      ps.Draw(modelShader);
    }
  }
  if (showAxes) {
    axisShader->Use();
    sceneAxes.Draw();
  }
}

void Scene::updateProjectionMatrix(int w, int h) {
  projectionMatrix =
      glm::perspective(camera.fov, float(w) / float(h), nearPlane, farPlane);
}

void Scene::handle_cameraZoomChange(float delta) {
  camera.processWheelMovement(delta);
}

void Scene::handle_cameraReset() {
  camera.resetCamera();
  light.changeLightDirection(camera.getPosition());
}

void Scene::initializeScene() {
  loadMesh("../Models/Small/test.obj");
  loadPointSphere();
  // loadMesh("../Models/cylinder.obj");
  // loadMesh("../Models/Small/coctel.obj");
  // loadMesh("../Models/Small/Wrong by assimp/stretched cube.obj");
  // loadMesh("../Models/teapot.obj");
  // loadMesh("../Models/icosahedron.obj");

  PS.setPosition(0, 0, 0);
}

void Scene::loadPointSphere() { PS.load("../Models/Small/icosahedron.obj"); }

void Scene::handle_segmentSelection(float mousePosX, float mousePosY,
                                    int windowWidth, int windowHeight) {

  Ray_intersection intersection;
  bool intersectionFound = rayIntersectsPolyhedron(
      mousePosX, mousePosY, windowWidth, windowHeight, intersection);
  if (intersectionFound) {
    P.handleSegmentSelection(intersection);
  }
}

void Scene::handle_showSegments() { P.handleShowSegments(); }

void Scene::handle_segmentContraction() {}

void Scene::handle_meshContraction() {
  contractMesh();
  if (m_showPointSpheresOnVertices)
    updatePointSpheresOnVerticesPositions();
}

void Scene::handle_meshConnectivitySurgery() {
  // I suppose mesh has already been contracted
  executeMeshConnectivitySurgery();
  if (m_showPointSpheresOnVertices)
    updatePointSpheresOnVerticesPositions();
}

void Scene::handle_meshInflation() { P.handleInflation(); }

void Scene::handle_meshDeflation() { P.handleDeflation(); }

void Scene::loadMesh(std::__cxx11::string filename) {
  camera.resetCamera();
  light.changeLightDirection(camera.getPosition());

  P = Mesh{};
  P.load(filename);
  EM = P.M; // copy contents to another object which will be edited.
  MC = MeshContractor(EM);

  PS.updateRadius(P.M);
}

void Scene::handle_axesStateChange(bool value) { showAxes = value; }

bool Scene::rayIntersectsPolyhedron(const int &mouseX, const int &mouseY,
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

  CGAL::Ray_3<Kernel> ray(Kernel::Point_3(camPos.x, camPos.y, camPos.z),
                          Kernel::Direction_3(ray_wor.x, ray_wor.y, ray_wor.z));

  intersection = P.intersects(ray);
  if (intersection) {
    return true;
  }
  return false;
}

void Scene::updatePointSpheresOnVerticesPositions() {
  m_pointSpheresOnVertices.clear();
  for (const auto v : P.M.vertices()) {
    PointSphere tempPS = PS;
    auto p(P.M.point(v));
    tempPS.setPosition(p);
    m_pointSpheresOnVertices.push_back(tempPS);
  }
}

void Scene::handle_mouseMovement(const QVector2D &mouseDV) {
  camera.processMouseMovement(mouseDV);
  light.changeLightDirection(camera.getPosition());
}

void Scene::handle_meshVerticesStateChange(int state) {
  m_showPointSpheresOnVertices = bool(state);
  updatePointSpheresOnVerticesPositions();
}

void Scene::contractMesh() {
  MC->get()->calculateSkeleton();
  P.updateVertices(EM);
}

void Scene::executeMeshConnectivitySurgery() {
  ConnectivitySurgeon CS(MC->get()->getContractedMesh());
}

void Scene::setSceneUniforms(Shader *modelShader, Shader *axisShader) {
  modelShader->Use();
  light.setUniforms(modelShader);
  camera.setUniforms(modelShader);
  setProjectionMatrixUniform(modelShader);

  axisShader->Use();
  setProjectionMatrixUniform(axisShader);
  glUniformMatrix4fv(glGetUniformLocation(axisShader->programID, "model"), 1,
                     GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
  camera.setUniforms(axisShader);
}

void Scene::setProjectionMatrixUniform(Shader *shader) {
  glUniformMatrix4fv(glGetUniformLocation(shader->programID, "projection"), 1,
                     GL_FALSE, glm::value_ptr(projectionMatrix));
}

// void Scene::contractSegment(int segmentIndex) {

//  MeshSegment S = P.getMeshSegment(segmentIndex); // TODO:create something
//                                                  // better than the
//                                                  // MeshSegment class

//  MeshContractor SC = MeshContractor(S.M);
//  CGALSurfaceMesh contractedMesh = SC.getContractedMesh();

//  for (auto v : contractedMesh.vertices()) {
//    CGALSurfaceMesh::Point p = contractedMesh.point(v);
//    P.vertices[S.vertexCorrespondence[size_t(v)]].Position =
//        glm::vec3(p.x(), p.y(), p.z());
//  }
//  // for (auto v : contractedMesh.vertices()) {
//  //  CGALSurfaceMesh::Point p = contractedMesh.point(v);
//  //  P.M.point(CGALSurfaceMesh::Vertex_index(
//  //      S.vertexCorrespondence[contractedMeshVertexIndex])) =
//  //      CGALSurfaceMesh::Point((p.x() - P.centerOfMass.x) / P.maxDim,
//  //                             (p.y() - P.centerOfMass.y) / P.maxDim,
//  //                             (p.z() - P.centerOfMass.z) / P.maxDim);
//  //  contractedMeshVertexIndex++;
//  //  P.vertices[S.vertexCorrespondence[contractedMeshVertexIndex]].Position
//  =
//  //      glm::vec3(p.x(), p.y(), p.z())contractMesh();;
//  //}
//  P.centerOfMass = meshMeasuring::findCenterOfMass(P.vertices);
//  P.maxDim = meshMeasuring::findMaxDimension(P.vertices);
//  P.normalizeMeshViaModelMatrix();
//}

// void setIntersectionSphereUniforms(Shader *shader) {
//  shader->Use();
//  intersectionSphere.setUniforms(shader);
//}
