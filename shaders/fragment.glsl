#version 330 core
in vec3 Normal;
in vec3 FragPos;
out vec4 color;

struct Light {
    //vec3 position;
    vec3 direction;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

uniform Light light;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

uniform Material material;

uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 viewPos;

void main()
{
    //ambient
    vec3 ambient=light.ambient*material.ambient; //component wise multiplication

    //diffuse
    vec3 norm=normalize(Normal);
    vec3 lightDir=normalize(-light.direction);
    float diff=max(dot(norm,lightDir),0.0);
    vec3 diffuse=light.diffuse*(diff*material.diffuse);

    //specular
    float specularStrength=0.5f;
    vec3 viewDir=normalize(viewPos-FragPos);
    vec3 reflectDir=reflect(-lightDir,norm);
    float spec=pow(max(dot(viewDir,reflectDir),0.0),material.shininess);
    vec3 specular=light.specular*(spec*material.specular);

    vec3 result=ambient+diffuse+specular;
    color=vec4(result,1.0f);
}
