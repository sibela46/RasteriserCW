#ifndef LIGHTS_H
#define LIGHTS_H

#include <glm/glm.hpp>
#include <vector>

using namespace std;
using namespace glm;

struct Light {
  vec4 lightPos;
  vec3 lightColour;
  vec3 lightPower;
  vec3 indirectLightPowerPerArea;
};

vec3 lightYellow ( 1.f, 1.f, 0.8f );

void defineLights(vector<vec4> lightPositions, vector<Light>& lights) {

  for (size_t i = 0; i < lights.size(); i++) {
    lights[i].lightPos = lightPositions[i];
    lights[i].lightColour = lightYellow;
    lights[i].lightPower = 14.1f*lightYellow;
    lights[i].indirectLightPowerPerArea = 0.2f*vec3( 1, 1, 1 );
  }

}

#endif
