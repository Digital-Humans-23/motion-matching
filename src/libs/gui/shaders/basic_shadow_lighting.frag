#version 330 core

out vec4 FragColor;
uniform vec3 objectColor;
uniform float alpha;
uniform sampler2DShadow shadowMap;
uniform float bias;
#include "compute_shading.frag"

float CalcShadowFactor(vec4 LightSpacePos)
{
	LightSpacePos.xyz = 0.5*LightSpacePos.xyz+0.5;
	float visibility=textureProj(shadowMap, vec4(LightSpacePos.xy,  LightSpacePos.z-bias,LightSpacePos.w));

	return visibility;
}

float CalcShadowFactorSoft(vec4 LightSpacePos)
{
	float vis = 0.0;
	float weights[5] = float[5](0.1, 0.2, 0.4, 0.2, 0.1);
	for(int i = 0; i < 5; i++) {
		for(int j=0; j < 5; j++){
			vec4 p = LightSpacePos + vec4(0.002 * (i - 2), 0.002 * (j - 2), 0.0, 0.0);
			vis = vis +  weights[i] * weights[j] * CalcShadowFactor(p);
		}
	}
	return vis;
}

vec3 computeShadowMapShading(){
	vec3 ambient = computeAmbientComponent();
	vec3 diffuse = computeDiffuseComponent();
	vec3 specular = computeSpecularComponent();

	float shadowFactor = CalcShadowFactorSoft(lightSpacePos);
	return ambient + shadowFactor * (diffuse + specular);
}

void main()
{
	vec3 result =  computeShadowMapShading() * objectColor;
	FragColor = vec4(result, alpha);
} 
