#ifndef SPHERE_H
#define SPHERE_H

#include <glm/glm.hpp>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <string>
#include "Global.h"

#define PI        3.14159265358979323846264338327950288   /* pi */

void DrawSphere(screen* screen) {

  float stepI = PI/50;
  float stepJ = PI/100;
  float r = 20;

  for (float i = -PI; i < PI; i += stepI) {
    for (float j = -PI/2; j < PI/2; j += stepJ) {
      float x = r * sin(i) * cos(j);
      float y = r * sin(i) * sin(j);
      float z = r * cos(i);

      x /= 100;
      y /= 100;
      int projX = focalLength * (x / z) + SCREEN_WIDTH / 2;
      int projY = focalLength * (y / z) + SCREEN_HEIGHT / 2;
      PutPixelSDL(screen, projX, projY, vec3(0,1,0));
    }
  }
}

#endif
