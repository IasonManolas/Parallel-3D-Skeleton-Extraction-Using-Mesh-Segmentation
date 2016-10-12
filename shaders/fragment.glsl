#version 330 core
//in vec4 vertexColor;
in vec2 outTexCoord;

out vec4 color;

uniform sampler2D ourTexture1;
uniform sampler2D ourTexture2;

void main()
{
    color =mix(texture(ourTexture1,outTexCoord),texture(ourTexture2,outTexCoord),0.2f);

//    color=vec4(1.0f,0.0f,0.0f,0.0f);
}
