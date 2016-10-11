#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <iostream>
class Shader
{
public:
    GLuint programID;
    Shader(const GLchar* vertexShaderSource,const GLchar* fragmentShaderSource)
    {
            //Vertex Shader
        GLuint vertexShader=glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader,1,&vertexShaderSource,NULL);
        glCompileShader(vertexShader);

        //Check for compile time errors
        GLint success;
        GLchar infoLog[512];
        glGetShaderiv(vertexShader,GL_COMPILE_STATUS,&success);
        if(!success)
        {
            glGetShaderInfoLog(vertexShader,512,NULL,infoLog);
            std::cout<<"ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"<<infoLog<<std::endl;
        }

        //Fragment Shader
        GLuint fragmentShader=glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader,1,&fragmentShaderSource,NULL);
        glCompileShader(fragmentShader);

        //Check for compile time errors
        glGetShaderiv(fragmentShader,GL_COMPILE_STATUS,&success);
        if(!success)
        {
            glGetShaderInfoLog(fragmentShader,512,NULL,infoLog);
            std::cout<<"ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"<<infoLog<<std::endl;
        }

        //Link shaders
        programID=glCreateProgram();
        glAttachShader(programID,vertexShader);
        glAttachShader(programID,fragmentShader);
        glLinkProgram(programID);

        //Check for linking errors
        glGetProgramiv(programID,GL_LINK_STATUS,&success);
        if(!success)
        {
            glGetProgramInfoLog(programID,512,NULL,infoLog);
            std::cout<<"ERROR::SHADER::PROGRAM::LINKING_FAILED\n"<<infoLog<<std::endl;
        }
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

    }
    Shader()
    {

    }
};

#endif // SHADER_H
