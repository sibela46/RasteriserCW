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
};

vec3 lightYellow ( 1.f, 1.f, 0.8f );

void defineLights(vec4 averageLightPos, vector<Light>& lights) {

  for (size_t i = 0; i < lights.size(); i++) {
    lights[i].lightPos = averageLightPos;
    lights[i].lightColour = lightYellow;
    lights[i].lightPower = 2.1f*lightYellow;
  }
}

#endif
