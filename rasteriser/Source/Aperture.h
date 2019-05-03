#ifndef APERTURE_H
#define APERTURE_H

#include <glm/glm.hpp>
#include <vector>
#include "Global.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

void InterpolateCircles( vec2 start, vec2 end, vector<vec2>& result );

void LoadHexagonVertices(vector<vec4> &vertices, float x, float y) {
  vec4 A( x - 0.45, y + 0.65, -2.1f, 1.0 ); // v0 - top left
  vec4 B( x + 0.45, y + 0.65, -2.1f, 1.0 ); // v1
  vec4 C( x + 0.75, y, -2.1f, 1.0 ); // v2
  vec4 D( x + 0.45, y - 0.65, -2.1f, 1.0 ); // v3
  vec4 E( x - 0.45, y - 0.65, -2.1f, 1.0 ); // v4
  vec4 F( x - 0.75, y, -2.1f, 1.0 ); // v5 - left middle

  vertices.push_back(A);
  vertices.push_back(B);
  vertices.push_back(C);
  vertices.push_back(D);
  vertices.push_back(E);
  vertices.push_back(F);
}

void LoadApertureHexagon(vector<Triangle> &hexagon, vec2 centrePoint, float scale, vec3 color) {
  vector<vec4> vertices;
  centrePoint *= scale;
  LoadHexagonVertices(vertices, centrePoint.x, centrePoint.y);

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].x *= scale;
    vertices[i].y *= scale;
  }

  Triangle one = Triangle(vertices[0], vertices[1], vertices[3], color, "ghost");
  Triangle two = Triangle(vertices[0], vertices[3], vertices[4], color, "ghost");
  Triangle three = Triangle(vertices[1], vertices[2], vertices[3], color, "ghost");
  Triangle four = Triangle(vertices[0], vertices[4], vertices[5], color, "ghost");

  hexagon.push_back(one);
  hexagon.push_back(two);
  hexagon.push_back(three);
  hexagon.push_back(four);

}

void DrawCircle(screen * screen, int32_t centreX, int32_t centreY, int32_t radius, vec3 color)
{
   const int32_t diameter = (radius * 2);

   int32_t x = (radius - 1);
   int32_t y = 0;
   int32_t tx = 1;
   int32_t ty = 1;
   int32_t error = (tx - diameter);

   while (x >= y)
   {
      //  Each of the following renders an octant of the circle
      int pixels = 0;
      vec2 start = vec2(centreX, centreY);
      vec2 end = vec2(centreX + x, centreY - y);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      vector<vec2> line( pixels );
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX + x, centreY + y);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX - x, centreY - y);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX - x, centreY + y);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX + y, centreY - x);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX + y, centreY + x);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX - y, centreY - x);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      end = vec2(centreX - y, centreY + x);
      pixels = glm::max( abs(start.x - end.x), abs(start.y - end.y) ) + 1;
      InterpolateCircles( start, end, line );

      for (int i = 0; i < pixels; i++) {
        PutPixelSDL(screen, line[i].x, line[i].y, color);
      }

      if (error <= 0)
      {
         ++y;
         error += ty;
         ty += 2;
      }

      if (error > 0)
      {
         --x;
         tx += 2;
         error += (tx - diameter);
      }
   }
}

void InterpolateCircles( vec2 start, vec2 end, vector<vec2>& result ) {
  int N = result.size();
  float stepX = (end.x-start.x) / float(std::max(N-1,1));
  float stepY = (end.y-start.y) / float(std::max(N-1,1));

  float currentX (start.x);
  float currentY (start.y);
  for( int i=0; i<N; ++i ) {
    result[i].x = round(currentX);
    result[i].y = round(currentY);
    currentX += stepX;
    currentY += stepY;
  }
}

#endif
