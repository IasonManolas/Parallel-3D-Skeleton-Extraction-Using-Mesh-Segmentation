#version 330 core
struct Material {
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

struct Light {
	//    vec3 position;
	vec3 direction;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

in vec3 FragPos;
in vec3 Normal;
in vec3 Color;

out vec4 color;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;
uniform float alpha;

void main()
{
	vec3 ambient = vec3(0.0f,0.0f,0.0f);
	// Diffuse
	vec3 norm = normalize(Normal);
	vec3 lightDir = normalize(-light.direction);
	float diff = max(dot(norm, lightDir), 0.0);
	vec3 diffuse = light.diffuse * diff *Color;

	// Specular
	vec3 viewDir = normalize(viewPos - FragPos);
	vec3 reflectDir = reflect(-lightDir, norm);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.specular * spec *vec3(0.5f,0.5f,0.5f);

	vec3 result = ambient+diffuse+specular;

	color = vec4(result, alpha);
}

