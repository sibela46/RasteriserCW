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

struct ClipPlane {
  vec3 normal;
  vec3 point;
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

// Clipping variables
// 20
const float sinatb = 0.342020;
const float cosatb = 0.939692;

// 25
const float sina = 0.422618;
const float cosa = 0.906308;

ClipPlane topPlane = {vec3(0, -cosatb, sinatb), vec3(0, 0, 0 + cameraPos.z)};
ClipPlane bottomPlane = {vec3(0, cosatb, sinatb), vec3(0, 0, 0 + cameraPos.z)};
ClipPlane leftPlane = {vec3(cosa, 0, sina), vec3(0, 0, 0 + cameraPos.z)};
ClipPlane rightPlane = {vec3(-cosa, 0, sina), vec3(0, 0, 0 + cameraPos.z)};
ClipPlane nearPlane = {vec3(0, 0, 1), vec3(0, 0, 1.5f)};
ClipPlane farPlane = {vec3(0, 0, -1), vec3(0, 0, 10.f)};
vector<ClipPlane> planes = {topPlane, bottomPlane, leftPlane, rightPlane, nearPlane, farPlane};

/* Light source variables */
int lightsStartIndex = 0;
int numOfHexagons = 8;
vec3 indirectLightPowerPerArea = 0.2f*vec3( 1.f, 1.f, 1.f );
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
