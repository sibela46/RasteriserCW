#ifndef LIGHT_H
#define LIGHT_H

#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <string>
#include "TestModelH.h"

using namespace std;
using glm::vec3;
using glm::vec4;

void LoadLightModel(vector<Triangle>& model) {
  int prevIndex = model.size();
  vector<vec4> temp_vertices;
  vector<int> elements;
  vec3 gray( 0.75f, 0.75f, 0.75f );

  std::ifstream infile("lamp.obj");
  if (!infile) {
    cerr << "Cannot open file" << endl; exit(1);
  }
  std::string line;

  while (std::getline(infile, line)) {
    if (line.substr(0,2) == "v ") {
      istringstream s(line.substr(2));
      glm::vec4 v; s >> v.x; s >> v.y; s >> v.z; v.w = 1.0f;
      temp_vertices.push_back(v);
    } else if (line.substr(0,2) == "f ") {
      istringstream s(line.substr(2));
      string a,b,c;
      s >> a; s >> b; s >> c;

      int d,e,f;
      d = stoi(a.substr(0, a.find("/")));
      e = stoi(b.substr(0, b.find("/")));
      f = stoi(c.substr(0, c.find("/")));

      d--; e--; f--;
      model.push_back(Triangle(temp_vertices[d], temp_vertices[e], temp_vertices[f], gray));
    } else {
      /* Ignoring any other line */
    }
  }

  for( size_t i=prevIndex; i<model.size(); ++i )
  {
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

    model[i].v0 -= vec4(0,-1,0,0);
		model[i].v1 -= vec4(0,-1,0,0);
		model[i].v2 -= vec4(0,-1,0,0);

    model[i].ComputeNormal();
    model[i].ReverseNormal();
  }
}

#endif
