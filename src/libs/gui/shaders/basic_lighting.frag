#version 330 core

#include "compute_shading.frag"

struct Material
{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

struct DirLight {
	vec3 direction;
	vec3 lightColor;
};

out vec4 FragColor;
uniform vec3 objectColor;
uniform float alpha;
uniform sampler2D texture_diffuse1;
in vec2 TexCoords;
uniform Material material;
uniform bool use_textures;
uniform bool use_material;

vec3 CalcDirLight(DirLight light, vec3 normal) {
	vec3 lightDir = normalize(-light.direction);
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuse  = 0.8  * diff * light.lightColor;
	return diffuse;
}

void main()
{
	vec3 viewDir = normalize(camPos - FragPos);
	vec3 norm_ = normalize(Normal);

	DirLight newLight;
	newLight.direction = -viewDir;
	newLight.lightColor = vec3(0.5, 0.5, 0.5);

	if(use_textures == true){
		vec3 color = computeBasicShading();
		color += CalcDirLight(newLight, norm_)* color;
		FragColor = vec4(color, alpha) * texture(texture_diffuse1, TexCoords);
	} else if(use_material == true) {
		vec3 ambient = material.ambient * computeAmbientComponent();
		vec3 diffuse = material.diffuse * computeDiffuseComponent();
		vec3 specular = material.specular * computeSpecularComponent();
		vec3 color = ambient + diffuse + specular;
		color += CalcDirLight(newLight, norm_)* color;
		FragColor = vec4(color, alpha);
	} else {
		vec3 color = computeBasicShading() * objectColor;
		color += CalcDirLight(newLight, norm_)* color;
		FragColor = vec4(color, alpha);
	}
} 
