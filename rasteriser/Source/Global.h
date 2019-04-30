#ifndef GLOBAL_H
#define GLOBAL_H

#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include <stdint.h>

SDL_Event event;

#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 720
#define FULLSCREEN_MODE false

struct Material {
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
  float shininess;
};

struct Pixel {
  int x;
  int y;
  float zinv;
  vec4 pos3d;
  string object;
};

struct Vertex {
  vec4 position;
  string object;
};

mat4 M;
float xaw = 0.f;
float yaw = 0.f;
float zaw = 0.f;
int pass = 0;
vec4 prevCameraPos = vec4(0.f, 0.f, -2.5f, 1.f);
vec4 cameraPos = vec4(0.f, 0.f, -2.5f, 1.f);
const vec4 initialCameraPos = cameraPos;
float focalLength = SCREEN_HEIGHT/2;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
float shadowMap[SCREEN_HEIGHT][SCREEN_WIDTH];
vec3 imageBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
/* Light source variables */
int lightsStartIndex = 0;
vec3 indirectLightPowerPerArea = 0.7f*vec3( 1.f, 1.f, 1.f );
vec4 currentNormal;
vec3 currentReflectance;
// Defines colors:
vec3 red(    0.75f, 0.15f, 0.15f );
vec3 yellow( 0.75f, 0.75f, 0.15f );
vec3 green(  0.15f, 0.75f, 0.15f );
vec3 cyan(   0.15f, 0.75f, 0.75f );
vec3 blue(   0.15f, 0.15f, 0.75f );
vec3 purple( 0.75f, 0.15f, 0.75f );
vec3 white(  0.75f, 0.75f, 0.75f );


#endif
