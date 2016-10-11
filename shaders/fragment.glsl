#version 330 core
//in vec4 vertexColor;
in vec3 vertexColor;
out vec4 color;

void main()
{
    color =vec4(vertexColor,1.0);
}
