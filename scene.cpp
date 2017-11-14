#include "scene.h"

void Scene::Draw(Shader *modelShader, Shader *axisShader, Shader *edgeShader) {
  setSceneUniforms(modelShader, axisShader, edgeShader);
  M.handle_drawing(modelShader, edgeShader);
  if (showAxes) {
    axisShader->Use();
    sceneAxes.Draw();
  }
}
void Scene::setSkeletonShaderUniforms(Shader *shader) {
  shader->Use();
  camera.setUniforms(shader);
  setProjectionMatrixUniform(shader);
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
  // loadMesh("../Models/tyra.obj");
  loadPointSphere();
  PS.setPosition(0, 0, 0);
  // loadMesh("../Models/Small/test.obj");

  // loadMesh("../Models/tyra_rightFoot.off");
  loadMesh("../Models/bunny_low.obj");
  // loadMesh("../Models/cylinder.obj");
  // loadMesh("../Models/Small/coctel.obj");
  // loadMesh("../Models/Small/Wrong by assimp/stretched cube.obj");
  // loadMesh("../Models/teapot.obj");
  // loadMesh("../Models/icosahedron.obj");
}

void Scene::loadPointSphere() { PS.load("../Models/Small/icosahedron.obj"); }

void Scene::handle_segmentSelection(float mousePosX, float mousePosY,
                                    int windowWidth, int windowHeight) {

  Ray_intersection intersection;
  bool intersectionFound = rayIntersectsPolyhedron(
      mousePosX, mousePosY, windowWidth, windowHeight, intersection);
  if (intersectionFound) {
    M.handle_segmentSelection(intersection);
  }
}

void Scene::handle_showSegments() { M.handle_showSegments(); }

void Scene::handle_segmentContraction(bool automatic) {
  M.handle_segmentContraction(automatic);
}

void Scene::handle_meshContraction(bool automatic) {
  M.handle_meshContraction(automatic);
}

void Scene::handle_meshConnectivitySurgery() {
  M.handle_meshConnectivitySurgery();
}

void Scene::handle_meshRefinementEmbedding() {
  M.handle_meshRefinementEmbedding();
}

void Scene::handle_segmentRefinementEmbedding() {}

void Scene::handle_segmentConnectivitySurgery() {
  M.handle_segmentConnectivitySurgery();
}

void Scene::handle_meshContractionReversing() {
  M.handle_meshContractionReversing();
}

void Scene::handle_meshInflation() { M.handle_inflation(); }

void Scene::handle_meshDeflation() { M.handle_deflation(); }

void Scene::loadMesh(std::__cxx11::string filename) {
  camera.resetCamera();
  light.changeLightDirection(camera.getPosition());

  // M = Mesh(PS);
  M.load(filename);
  PS.updateRadius(M.getM());
  M.setPointSphere(PS);
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
  // NDCSpace -> clipSpace
  glm::vec4 ray4_clipSpace(ndcs.x, ndcs.y, ndcs.z, 1.0);
  // clipSpace -> eyeSpace
  glm::vec4 ray4_eyeSpace = glm::inverse(projectionMatrix) * ray4_clipSpace;
  ray4_eyeSpace = glm::vec4(ray4_eyeSpace.x, ray4_eyeSpace.y, -1.0, 0.0);
  // eyeSpace -> worldSpace
  glm::vec4 ray4_worldSpace =
      glm::inverse(camera.getViewMatrix()) * ray4_eyeSpace;

  // worldSpace -> modelSpace
  glm::vec4 ray4_modelSpace =
      glm::inverse(M.getModelMatrix()) * ray4_worldSpace;

  glm::vec3 ray3_modelSpace(ray4_modelSpace.x, ray4_modelSpace.y,
                            ray4_modelSpace.z);
  ray3_modelSpace = glm::normalize(ray3_modelSpace);

  glm::vec4 camPos4_worldSpace = glm::vec4(camPos, 1.0);
  glm::vec4 camPos4_modelSpace =
      glm::inverse(M.getModelMatrix()) * camPos4_worldSpace;
  glm::vec3 camPos3_modelSpace(camPos4_modelSpace.x, camPos4_modelSpace.y,
                               camPos4_modelSpace.z);

  // glm ray -> CGAL ray
  CGAL::Ray_3<Kernel> ray_modelSpace(
      Kernel::Point_3(camPos3_modelSpace.x, camPos3_modelSpace.y,
                      camPos3_modelSpace.z),
      Kernel::Direction_3(ray3_modelSpace.x, ray3_modelSpace.y,
                          ray3_modelSpace.z));

  intersection = M.intersects(ray_modelSpace);

  if (intersection) {
    return true;
  }
  return false;
}

void Scene::handle_mouseMovement(const QVector2D &mouseDV) {
  camera.processMouseMovement(mouseDV);
  light.changeLightDirection(camera.getPosition());
}

void Scene::handle_meshVerticesStateChange(int state) {
  M.handle_showVerticesStateChange(state);
  // updatePointSpheresOnVerticesPositions();
}

void Scene::handle_saveModel(const std::__cxx11::string destinationDirectory) {
  M.handle_saveModel(destinationDirectory);
}

void Scene::handle_saveSegment(
    const std::__cxx11::string destinationDirectory) {
  M.handle_saveSegment(destinationDirectory);
}

void Scene::handle_clearSkeleton() { M.handle_clearSkeleton(); }

void Scene::setSceneUniforms(Shader *modelShader, Shader *axisShader,
                             Shader *edgeShader) {
  modelShader->Use();
  light.setUniforms(modelShader);
  camera.setUniforms(modelShader);
  setProjectionMatrixUniform(modelShader);

  edgeShader->Use();
  camera.setUniforms(edgeShader);
  setProjectionMatrixUniform(edgeShader);

  axisShader->Use();
  camera.setUniforms(axisShader);
  setProjectionMatrixUniform(axisShader);
  glUniformMatrix4fv(glGetUniformLocation(axisShader->programID, "model"), 1,
                     GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
}

void Scene::setProjectionMatrixUniform(Shader *shader) {
  glUniformMatrix4fv(glGetUniformLocation(shader->programID, "projection"), 1,
                     GL_FALSE, glm::value_ptr(projectionMatrix));
}
