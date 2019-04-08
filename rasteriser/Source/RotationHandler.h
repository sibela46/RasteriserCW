#ifndef ROTATION_H
#define ROTATION_H

#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <vector>
#include <stdint.h>
#include <sstream>
#include <string>

using namespace std;
using glm::mat4;

void RotationMatrixByAngle(float xangle, float yangle, float zangle, mat4& R) {
  R =  { cos(yangle)*cos(zangle), -cos(yangle)*sin(zangle) + sin(xangle)*sin(yangle)*cos(zangle), sin(xangle)*sin(zangle) + cos(xangle)*sin(yangle)*cos(zangle), 0,
  			cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), -sin(xangle)*cos(zangle) + cos(xangle)*sin(yangle)*sin(zangle), 0,
  			-sin(yangle)        , sin(xangle)*cos(yangle)                              , cos(xangle)*cos(yangle)                             , 0,
  			0                , 0                                              , 0                                             , 1 };
}

#endif
