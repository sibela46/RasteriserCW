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
  vec3 shadow;
};

struct Vertex {
  vec4 position;
  Material material;
};

Material basicShader = { vec3(1.0f, 0.5f, 0.31f), vec3(1.0f, 0.5f, 0.31f), vec3(0.5f, 0.5f, 0.5f), 32.0f };

float xaw = 0.f;
float yaw = 0.f;
float zaw = 0.f;
vec4 prevCameraPos = vec4(0.f, 0.f, -2.5f, 1.f);
vec4 cameraPos = vec4(0.f, 0.f, -2.5f, 1.f);
float focalLength = SCREEN_HEIGHT/2;
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
float shadowMap[SCREEN_HEIGHT][SCREEN_WIDTH];
/* Light source variables */
vec4 lightPos(0, 0.1f, -0.7f, 1.f);
vector<Light> lights(5);
vec4 currentNormal;
vec3 currentReflectance;


#endif
