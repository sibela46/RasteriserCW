#ifndef APERTURE_H
#define APERTURE_H

#include <glm/glm.hpp>
#include <vector>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

void LoadHexagonVertices(vector<vec4> &vertices) {
  vec4 A( -0.5, 0.65, -2, 1.0 ); // v0 - top left
  vec4 B( 0.5, 0.65, -2, 1.0 ); // v1
  vec4 C( 0.75, 0, -2, 1.0 ); // v2
  vec4 D( 0.5, -0.65, -2, 1.0 ); // v3
  vec4 E( -0.5, -0.65, -2, 1.0 ); // v4
  vec4 F( -0.75, 0, -2, 1.0 ); // v5 - left middle

  vertices.push_back(A);
  vertices.push_back(B);
  vertices.push_back(C);
  vertices.push_back(D);
  vertices.push_back(E);
  vertices.push_back(F);
}

void LoadApertureHexagon(vector<Triangle> &hexagon, float scale, float offset) {
  vector<vec4> vertices;
  LoadHexagonVertices(vertices);

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].y += (offset);
    vertices[i].x *= scale;
    vertices[i].y *= scale;
  }

  Triangle one = Triangle(vertices[0], vertices[1], vertices[3], vec3(1, 1, 0.5));
  Triangle two = Triangle(vertices[0], vertices[3], vertices[4], vec3(1, 1, 0.5));
  Triangle three = Triangle(vertices[1], vertices[2], vertices[3], vec3(1, 1, 0.5));
  Triangle four = Triangle(vertices[0], vertices[4], vertices[5], vec3(1, 1, 0.5));

  hexagon.push_back(one);
  hexagon.push_back(two);
  hexagon.push_back(three);
  hexagon.push_back(four);

}

#endif
