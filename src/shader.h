#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <iostream>
#include <fstream>
#include <sstream>
class Shader
{
public:
    GLuint programID;
    Shader(const GLchar* vertexPath,const GLchar* fragmentPath)
    {

        // 1. Retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        // ensures ifstream objects can throw exceptions:
        vShaderFile.exceptions (std::ifstream::badbit);
        fShaderFile.exceptions (std::ifstream::badbit);
        try
        {
            // Open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;
            // Read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            // close file handlers
            vShaderFile.close();
            fShaderFile.close();
            // Convert stream into string
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }
        const GLchar* vertexShaderSource = vertexCode.c_str();
        const GLchar * fragmentShaderSource = fragmentCode.c_str();

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

    Shader() //using this constructor in class GLWidget before opengl is initialized (?)
    {
    }

    void Use()
    {
        glUseProgram(programID);
    }
};

#endif // SHADER_H
