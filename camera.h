#ifndef CAMERA_H
#define CAMERA_H

#include "glm/ext.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //glm::lookAt()
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp> //glm::rotat

#include <QVector2D>
#include <QVector3D>

#include "shader.h"

class Camera {
public:
  Camera(const glm::vec3 &pos, const glm::vec3 &target, const glm::vec3 &up)
      : fov(45), position(pos), target(target), up(up), worldUp(up) {

    viewMatrix = glm::lookAt(position, target, up);
  }

  void calculateRotationMatrix(const QVector2D &mouseDV) {
    glm::vec3 rotationAxis(-mouseDV.y(), -mouseDV.x(),
                           0.0); // prepei na peristrefetai ws pros to
                                 // peristrammeno systhma syntetagmenwn kai oxi
                                 // ws pros to arxiko
    float angle = rotationAxis.length() / 100.0;
    rotationAxis = glm::normalize(rotationAxis);
    glm::quat rot = glm::angleAxis(angle, rotationAxis);
    rotationQ = rotationQ * rot;
    std::cout << glm::to_string(rotationQ) << std::endl;
    rotationMatrix = glm::toMat4(rotationQ);
  }

  void updateViewMatrix() { viewMatrix = glm::lookAt(position, target, up); }

  void updateVectors() {
    glm::vec3 initialPosition = glm::vec3(0, 0, distance);
    position = glm::vec3(rotationMatrix * glm::vec4(initialPosition, 0));

    //        look=glm::normalize(target-position);
    //        up=glm::vec3(rotationMatrix*glm::vec4(up,0));
    up = glm::vec3(rotationMatrix * glm::vec4(0, 1, 0, 0));
    //        right=glm::cross(look,up);
    std::cout << "CAM POS:X=" << position.x << " Y=" << position.y
              << " Z=" << position.z << std::endl;
  }

  void processMouseMovement(const QVector2D &mouseDV) {
    calculateRotationMatrix(mouseDV);
    updateVectors();
    updateViewMatrix();
  }

  void processWheelMovement(const int &wheelDelta) {
    if (wheelDelta > 0)
      distance -= 0.1;
    else
      distance += 0.1;
    updateVectors();
    updateViewMatrix();
  }
  glm::vec3 getPosition() const { return position; }
  void setPosition(const glm::vec3 &value) { position = value; }

  glm::mat4 getViewMat() const { return viewMatrix; }
  void setUniforms(Shader *shader) const {
    GLint viewPosLoc = glGetUniformLocation(shader->programID, "viewPos");
    if (viewPosLoc != -1)
      glUniform3f(viewPosLoc, position.x, position.y, position.z);
    glUniformMatrix4fv(glGetUniformLocation(shader->programID, "view"), 1,
                       GL_FALSE, glm::value_ptr(viewMatrix));
  }

  void resetCamera() {
    rotationQ = glm::quat();
    rotationMatrix = glm::mat4(1);
    distance = 3;
    updateVectors();
    updateViewMatrix();
  }

  float fov{45};

private:
  float distance{3};
  glm::vec3 position{glm::vec3(0, 0, distance)};
  glm::vec3 target{glm::vec3(0, 0, 0)};
  glm::vec3 look{glm::normalize(target - position)};
  glm::vec3 up{glm::vec3(0, 1, 0)};
  glm::vec3 worldUp{glm::vec3(0, 1, 0)};
  glm::vec3 right{glm::cross(look, up)};

  glm::mat4 viewMatrix{glm::mat4(1)};

  glm::quat rotationQ{
      glm::quat()}; // this quaternion represents the orientation of the cam?
  glm::mat4 rotationMatrix{glm::mat4(1)};
};
#endif // CAMERA_H
