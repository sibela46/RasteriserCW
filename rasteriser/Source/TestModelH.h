#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <string>
#include "RotationHandler.h"

#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 720

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;

// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 color;
	std::string object;

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color, std::string object )
		: v0(v0), v1(v1), v2(v2), color(color), object(object)
	{
		ComputeNormal();
	}

	void ReverseNormal() {
		normal *= -1;
		normal.w *= -1;
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
void LoadTestModel( std::vector<Triangle>& triangles )
{
	using glm::vec3;
	using glm::vec4;

	// Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

	triangles.clear();
	triangles.reserve( 5*2*3 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec4 A(L,0,0,1);
	vec4 B(0,0,0,1);
	vec4 C(L,0,L,1);
	vec4 D(0,0,L,1);

	vec4 E(L,L,0,1);
	vec4 F(0,L,0,1);
	vec4 G(L,L,L,1);
	vec4 H(0,L,L,1);

	// Floor:
	triangles.push_back( Triangle( C, B, A, green, "wall" ) );
	triangles.push_back( Triangle( C, D, B, green, "wall" ) );

	// Left wall
	triangles.push_back( Triangle( A, E, C, purple, "wall" ) );
	triangles.push_back( Triangle( C, E, G, purple, "wall" ) );

	// Right wall
	triangles.push_back( Triangle( F, B, D, yellow, "wall" ) );
	triangles.push_back( Triangle( H, F, D, yellow, "wall" ) );

	// Ceiling
	triangles.push_back( Triangle( E, F, G, cyan, "wall" ) );
	triangles.push_back( Triangle( F, H, G, cyan, "wall" ) );

	// Back wall
	triangles.push_back( Triangle( G, D, C, white, "wall" ) );
	triangles.push_back( Triangle( G, H, D, white, "wall" ) );

	// ---------------------------------------------------------------------------
	// Short block

	A = vec4(290,0,114,1);
	B = vec4(130,0, 65,1);
	C = vec4(240,0,272,1);
	D = vec4( 82,0,225,1);

	E = vec4(290,165,114,1);
	F = vec4(130,165, 65,1);
	G = vec4(240,165,272,1);
	H = vec4( 82,165,225,1);

	// Front
	triangles.push_back( Triangle(E,B,A,red, "wall") );
	triangles.push_back( Triangle(E,F,B,red, "wall") );

	// Front
	triangles.push_back( Triangle(F,D,B,red, "wall") );
	triangles.push_back( Triangle(F,H,D,red, "wall") );

	// BACK
	triangles.push_back( Triangle(H,C,D,red, "wall") );
	triangles.push_back( Triangle(H,G,C,red, "wall") );

	// LEFT
	triangles.push_back( Triangle(G,E,C,red, "wall") );
	triangles.push_back( Triangle(E,A,C,red, "wall") );

	// TOP
	triangles.push_back( Triangle(G,F,E,red, "wall") );
	triangles.push_back( Triangle(G,H,F,red, "wall") );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec4(423,0,247,1);
	B = vec4(265,0,296,1);
	C = vec4(472,0,406,1);
	D = vec4(314,0,456,1);

	E = vec4(423,330,247,1);
	F = vec4(265,330,296,1);
	G = vec4(472,330,406,1);
	H = vec4(314,330,456,1);

	// Front
	triangles.push_back( Triangle(E,B,A,blue, "wall") );
	triangles.push_back( Triangle(E,F,B,blue, "wall") );

	// Front
	triangles.push_back( Triangle(F,D,B,blue, "wall") );
	triangles.push_back( Triangle(F,H,D,blue, "wall") );

	// BACK
	triangles.push_back( Triangle(H,C,D,blue, "wall") );
	triangles.push_back( Triangle(H,G,C,blue, "wall") );

	// LEFT
	triangles.push_back( Triangle(G,E,C,blue, "wall") );
	triangles.push_back( Triangle(E,A,C,blue, "wall") );

	// TOP
	triangles.push_back( Triangle(G,F,E,blue, "wall") );
	triangles.push_back( Triangle(G,H,F,blue, "wall") );

	// ----------------------------------------------
	// Scale to the volume [-1,1]^3

	for( size_t i=0; i<triangles.size(); ++i )
	{
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec4(1,1,1,1);
		triangles[i].v1 -= vec4(1,1,1,1);
		triangles[i].v2 -= vec4(1,1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].v0.w = 1.0;
		triangles[i].v1.w = 1.0;
		triangles[i].v2.w = 1.0;

		triangles[i].ComputeNormal();
	}
}

void LoadBunnyModel( std::vector<Triangle>& model ) {

  vector<vec4> temp_vertices;
	int prevIndex = model.size();

  std::ifstream infile("bunny.obj");
  std::string line;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    char l;

    if (!(iss >> l)) { break; } // error
    if (l == 'v') {
      float a, b, c;
      if (!(iss >> a >> b >> c)) { break; }
      temp_vertices.push_back(vec4(a, b, c, 1.0f));
    } else if (l == 'f') {
      float a, b, c;
      if (!(iss >> a >> b >> c)) { break; }

      model.push_back(Triangle(temp_vertices[a-1], temp_vertices[b-1], temp_vertices[c-1], vec3(0.75, 0.75, 0.75), "bunny"));

    }
  }

  // ----------------------------------------------
  // Scale to the volume [-1,1]^3
	mat4 R;
	RotationMatrixByAngle(0.0f, -35.0f, 0.0f, R);

  for( size_t i=prevIndex; i<model.size(); ++i )
  {

		model[i].v0 = R * model[i].v0;
		model[i].v1 = R * model[i].v1;
		model[i].v2 = R * model[i].v2;

		model[i].v0 *= 5;
		model[i].v1 *= 5;
		model[i].v2 *= 5;

		model[i].v0 += vec4(1,1,1,1);
		model[i].v1 += vec4(1,1,1,1);
		model[i].v2 += vec4(1,1,1,1);

    model[i].v0 -= vec4(1,1,1,1);
    model[i].v1 -= vec4(1,1,1,1);
    model[i].v2 -= vec4(1,1,1,1);

    model[i].v0.x *= -1;
    model[i].v1.x *= -1;
    model[i].v2.x *= -1;

    model[i].v0.y *= -1;
    model[i].v1.y *= -1;
    model[i].v2.y *= -1;

		model[i].v0.w = 1.0;
		model[i].v1.w = 1.0;
		model[i].v2.w = 1.0;

		model[i].v0 += vec4(0.4,0.6,-0.3,0);
		model[i].v1 += vec4(0.4,0.6,-0.3,0);
		model[i].v2 += vec4(0.4,0.6,-0.3,0);

    model[i].ComputeNormal();
		model[i].ReverseNormal();
  }
}

#endif
