#version 330 core

layout (location = 0) in float x;
layout (location = 1) in float y;
layout (location = 2) in float z;

uniform mat4 MVP;
void main()
{
    gl_Position = MVP*vec4(x,y,z, 1.0f);
}
