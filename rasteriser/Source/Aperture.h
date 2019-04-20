#ifndef APERTURE_H
#define APERTURE_H

#include <glm/glm.hpp>
#include <vector>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

void LoadHexagonVertices(vector<vec4> &vertices, float x, float y) {
  vec4 A( x - 0.5, y + 0.65, -2, 1.0 ); // v0 - top left
  vec4 B( x + 0.5, y + 0.65, -2, 1.0 ); // v1
  vec4 C( x + 0.75, y, -2, 1.0 ); // v2
  vec4 D( x + 0.5, y - 0.65, -2, 1.0 ); // v3
  vec4 E( x - 0.5, y - 0.65, -2, 1.0 ); // v4
  vec4 F( x - 0.75, y, -2, 1.0 ); // v5 - left middle

  vertices.push_back(A);
  vertices.push_back(B);
  vertices.push_back(C);
  vertices.push_back(D);
  vertices.push_back(E);
  vertices.push_back(F);
}

void LoadApertureHexagon(vector<Triangle> &hexagon, vec2 centrePoint, float scale) {
  vector<vec4> vertices;
  centrePoint *= scale;
  LoadHexagonVertices(vertices, centrePoint.x, centrePoint.y);

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].x *= scale;
    vertices[i].y *= scale;
  }

  // Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

  Triangle one = Triangle(vertices[0], vertices[1], vertices[3], yellow);
  Triangle two = Triangle(vertices[0], vertices[3], vertices[4], white);
  Triangle three = Triangle(vertices[1], vertices[2], vertices[3], white);
  Triangle four = Triangle(vertices[0], vertices[4], vertices[5], yellow);

  hexagon.push_back(one);
  hexagon.push_back(two);
  hexagon.push_back(three);
  hexagon.push_back(four);

}

#endif
