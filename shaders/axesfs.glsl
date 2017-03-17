#version 330 core
in vec3 Color;

out vec4 color;
void main(void)
{
    color=vec4(Color,1.0f);
}
