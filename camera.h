#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //glm::lookAt()
#include <glm/gtx/rotate_vector.hpp> //glm::rotat
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <QVector2D>
#include <QVector3D>

#include "shader.h"

class Camera
{
public:
    Camera(){}
    Camera(const glm::vec3 &pos,const glm::vec3 &target,const glm::vec3 &up):fov(45.0f)
    {
        this->position=pos;
        this->target=target;
        this->worldUp=glm::normalize(up);
        this->up=worldUp;

        right=glm::normalize(glm::cross(look,worldUp));
        viewMatrix=glm::lookAt(position,target,up);

        rotationQ=glm::quat();
    }

    void processMouseMovement(const QVector2D& mouseDV)
    {
        glm::vec3 rotationAxis(mouseDV.y(),mouseDV.x(),0.0);
        float angle=rotationAxis.length()/200.0;
        rotationAxis=glm::normalize(rotationAxis);

        glm::quat rot=glm::angleAxis(angle,rotationAxis);
        rotationQ=rot*rotationQ;
        rotationMatrix=glm::toMat4(rotationQ);

        updateViewMatrix();
     }


    void processWheelMovement(const int& wheelDelta)
    {
        if(wheelDelta>0)
        position+=0.1f*glm::normalize(target-position);
        else
        position-=0.1f*glm::normalize(target-position);

        updateViewMatrix();
    }

    glm::vec3 getPosition() const
    {
        return position;
    }
    void setPosition(const glm::vec3 &value)
    {
        position=value;
    }

    glm::mat4 getViewMat() const
    {
       return viewMatrix;
    }
    void setUniforms(Shader* shader) const
    {
        GLuint viewPosLoc=glGetUniformLocation(shader->programID,"viewPos");
        if(viewPosLoc!=-1) glUniform3f(viewPosLoc,position.x,position.y,position.z);
        glUniformMatrix4fv(glGetUniformLocation(shader->programID,"view"),1,GL_FALSE,glm::value_ptr(viewMatrix));
    }

    float fov;
private:
    void updateViewMatrix()
    {
        viewMatrix=glm::lookAt(position,target,up);
        viewMatrix=viewMatrix*rotationMatrix;
    }

    glm::vec3 position;
    glm::vec3 target;

    glm::vec3 look;
    glm::vec3 worldUp;
    glm::vec3 up;
    glm::vec3 right;

    glm::mat4 viewMatrix;

    glm::quat rotationQ; //this quaternion represents the orientation of the cam?
    glm::mat4 rotationMatrix;
};
#endif // CAMERA_H
