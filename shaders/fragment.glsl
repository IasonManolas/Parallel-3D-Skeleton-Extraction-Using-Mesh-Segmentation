#version 330 core

in vec2 TexCoords;

//out vec4 color;

uniform sampler2D texture_diffuse1;

//void main()
//{
//    color = vec4(texture(texture_diffuse1, TexCoords));
//}
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct Light {
    //vec3 position;
    vec3 direction;

    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

in vec3 FragPos;
in vec3 Normal;

out vec4 color;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;

void main()
{
    // Ambient
    vec3 ambient = light.ambient*vec3(texture(texture_diffuse1,TexCoords)) ;

    // Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * vec3(texture(texture_diffuse1,TexCoords));

    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * spec *material.specular;

    vec3 result = ambient + diffuse + specular;
    color = vec4(result, 1.0f);
//    color = vec4(texture(texture_diffuse1, TexCoords));
}
