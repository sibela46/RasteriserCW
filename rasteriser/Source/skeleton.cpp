#include <iostream>
#include<fstream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include "TestModelH.h"
#include "Lightbulb.h"
#include "Lights.h"
#include "Aperture.h"
#include "Sphere.h"
#include "Global.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;
using glm::ivec2;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update(vector<Light>& lights);
void Draw(screen* screen, vector<Triangle> triangles, const vector<Light> lights, int lastIndex );
void VertexShader( const Vertex& vertices, Pixel& projPos );
void VertexShaderLight( const Vertex& vertex, Pixel& projPos );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( screen* screen, const vector<Light> lights, Pixel a, Pixel b, vec3 shadow );
void DrawPolygonEdges( screen* screen, const vector<Vertex>& vertices );
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void ComputeShadows( const vector<Vertex> vertices );
void TransformationMatrix( glm::mat4x4& M );
void DrawRows(screen* screen, const vector<Light> lights, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 shadow);
void DrawPolygon(screen* screen, const vector<Vertex>& vertices, const vector<Light> lights );
void PixelShader( screen* screen, const vector<Light> lights, const Pixel& pixel, vec3 shadow );
void getLensFlare(vector<vec2>& positions, vector<float>& scales, const vector<Light> lights);
void changeLensFlare(vector<vec2>& positions, vector<float>& scales, const vector<Light> lights);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);
  int lightsStartIndex = triangles.size();
  LoadLightModel(triangles);

  int lightSize = int((triangles.size() - lightsStartIndex)/100);
  vector<Light> lights(lightSize);
  vector<vec4> lightPositions;

  for (size_t i = 0; i < lightSize; i++) {
    /* Correct the light's position wrt our camera */
    int randIndex = rand() % (triangles.size()) + lightsStartIndex;
    vec4 toPush = triangles[randIndex].v0 - cameraPos;
    lightPositions.push_back(toPush);
  }

  vec4 averageLightPos = vec4(0, 0, 0, 0);
  for (size_t i = 0; i < lightSize; i++) {
    averageLightPos += lightPositions[i];
  }
  averageLightPos /= lightSize;

  int lightsEndIndex = triangles.size();
  // cout << averageLightPos.x << " " << averageLightPos.y << " " << averageLightPos.z << endl;
  defineLights(averageLightPos, lights);

  LoadBunnyModel(triangles);
  vector<vec2> randomPositions(6);
  vector<float> randomScales(6);
  getLensFlare(randomPositions, randomScales, lights);

  while ( Update(lights))
  {
      int lastIndex = triangles.size();
      randomPositions.clear();
      randomScales.clear();
      changeLensFlare(randomPositions, randomScales, lights);

      LoadApertureHexagon(triangles, randomPositions[0], randomScales[0], yellow);
      LoadApertureHexagon(triangles, randomPositions[1], randomScales[1], yellow);

      LoadApertureHexagon(triangles, randomPositions[2], randomScales[2], cyan);
      LoadApertureHexagon(triangles, randomPositions[3], randomScales[3], cyan);

      LoadApertureHexagon(triangles, randomPositions[4], randomScales[4], purple);
      LoadApertureHexagon(triangles, randomPositions[5], randomScales[5], purple);

      Draw(screen, triangles, lights, lastIndex);

      vec2 circlePoint = vec2(int(SCREEN_WIDTH/2 + randomPositions[2].x/5), int(SCREEN_HEIGHT/2 + randomPositions[2].y/5));
      DrawCircle(screen, circlePoint.x, circlePoint.y, randomScales[2]*800, cyan);
      DrawCircle(screen, circlePoint.x, circlePoint.y, randomScales[3]*800, cyan);

      for (size_t i = 0; i < lightSize; i++) {
        /* Correct the light's position wrt our camera */
        int lightX = focalLength * (lights[i].lightPos.x / lights[i].lightPos.z) + SCREEN_WIDTH / 2;
        int lightY = focalLength * (lights[i].lightPos.y / lights[i].lightPos.z) + SCREEN_HEIGHT / 2;

        PutPixelSDL(screen, lightX, lightY, vec3(1, 0, 0));
      }

      triangles.erase(triangles.begin() + lastIndex, triangles.end());
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle> triangles, const vector<Light> lights, int lastIndex)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  /* Clear depth and shadow buffer */
  for (int y = 0; y < SCREEN_HEIGHT; ++y) {
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
      depthBuffer[y][x] = 0;
      imageBuffer[y][x] = vec3(0, 0, 0);
      shadowMap[y][x] = numeric_limits<int>::max();
    }
  }
  /* Setup rotation matrix */
  mat4 M;
  TransformationMatrix(M);

  // /* First pass is for computing the depth buffer */
  // for (uint32_t i = 0; i < triangles.size(); i++) {
  //   vector<Vertex> vertices(3);
  //   vertices[0].position = triangles[i].v0;
  //   vertices[1].position = triangles[i].v1;
  //   vertices[2].position = triangles[i].v2;
  //   currentNormal = triangles[i].normal;
  //   for(int v = 0; v < 3; ++v) {
  //     if (i < lastIndex) {
  //       vertices[v].position = M * vertices[v].position;
  //     }
  //   }
  //   ComputeShadows( vertices );
  // }

  /* Second pass is for drawing the actual pixels */
  for( uint32_t i=0; i < triangles.size(); ++i ) {
      vector<Vertex> vertices(3);
      vertices[0].position = triangles[i].v0;
      vertices[1].position = triangles[i].v1;
      vertices[2].position = triangles[i].v2;
      currentNormal = triangles[i].normal;
      currentReflectance = triangles[i].color;
      for (int v = 0; v < 3; ++v) {
          if (i < lastIndex) vertices[v].position = M * vertices[v].position;
          vertices[v].object = triangles[i].object;
      }
      DrawPolygon( screen, vertices, lights );
  }
  /* Second pass is for drawing the actual pixels */
  for( uint32_t i=0; i < triangles.size(); ++i ) {
      vector<Vertex> vertices(3);
      vertices[0].position = triangles[i].v0;
      vertices[1].position = triangles[i].v1;
      vertices[2].position = triangles[i].v2;
      currentNormal = triangles[i].normal;
      currentReflectance = triangles[i].color;
      for (int v = 0; v < 3; ++v) {
          if (i < lastIndex) vertices[v].position = M * vertices[v].position;
          vertices[v].object = triangles[i].object;
      }
      DrawPolygon( screen, vertices, lights );
  }
}

/* Compute shadow given light source */
void ComputeShadows( const vector<Vertex> vertices ) {
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i ){
    VertexShader( vertices[i], vertexPixels[i] );
    int x = vertexPixels[i].x;
    int y = vertexPixels[i].y;

    if (vertexPixels[i].zinv < shadowMap[y][x]) {
      shadowMap[y][x] = vertexPixels[i].zinv;
    }
  }
}

void VertexShader( const Vertex& vertex, Pixel& projPos ) {
  vec4 temp = vertex.position - cameraPos; // * cam_rotation
  projPos.x = focalLength * (temp.x / temp.z) + SCREEN_WIDTH / 2;
  projPos.y = focalLength * (temp.y / temp.z) + SCREEN_HEIGHT / 2;
  projPos.zinv = focalLength * (1 / (temp.z));
  projPos.pos3d = temp;
  projPos.object = vertex.object;
}


void PixelShader( screen* screen, const vector<Light> lights, const Pixel& pixel, vec3 shadow ) {
    int x = pixel.x;
    int y = pixel.y;
    vec3 R = vec3(0, 0, 0);

    /* Compute illumination */
    for (size_t i = 0; i < lights.size(); i++) {
      float r = glm::distance(pixel.pos3d, lights[i].lightPos);
      float A = 4 * M_PI * r * r;
      vec4 r_hat = glm::normalize(lights[i].lightPos - pixel.pos3d);
      vec4 n = currentNormal;
      vec3 p = currentReflectance;
      vec3 P = lights[i].lightPower / A;
      vec3 D = P * std::max(glm::dot(n, r_hat), 0.0f);
      vec3 diffuse = p * D;

      R += diffuse;
    }

    vec3 green(  0.15f, 0.75f, 0.15f );
  	vec3 blue(   0.15f, 0.55f, 1.0f );
  	vec3 yellow( 0.75f, 0.75f, 0.15f );

    if ( pixel.zinv > depthBuffer[y][x] ) {
      depthBuffer[y][x] = pixel.zinv;
      if (pixel.object != "light") {
        PutPixelSDL( screen, x, y, R + currentReflectance*indirectLightPowerPerArea);
      }
      if (pixel.object != "ghost") {
        imageBuffer[y][x] = R + currentReflectance*indirectLightPowerPerArea;
      }
      if (pixel.object == "ghost") {
        PutPixelSDL( screen, x, y, imageBuffer[y][x] + currentReflectance*vec3(0.2,0.2,0.2));
      }
    }

}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ) {
  int N = result.size();
  float stepX = (b.x-a.x) / float(std::max(N-1,1));
  float stepY = (b.y-a.y) / float(std::max(N-1,1));
  float stepZ = (b.zinv-a.zinv) / float(std::max(N-1,1));
  vec4 stepI = (b.pos3d*b.zinv-a.pos3d*a.zinv) / float(std::max(N-1,1));
  float currentX (a.x);
  float currentY (a.y);
  float currentZ (a.zinv);
  vec4 currentI (a.pos3d*a.zinv);
  for( int i=0; i<N; ++i ) {
    result[i].x = round(currentX);
    result[i].y = round(currentY);
    result[i].zinv = currentZ;
    result[i].pos3d = currentI / currentZ;
    currentX += stepX;
    currentY += stepY;
    currentZ += stepZ;
    currentI += stepI;
  }
}

void DrawLineSDL( screen* screen, const vector<Light> lights, Pixel a, Pixel b, vec3 shadow ) {
  int pixels = glm::max( abs(a.x - b.x), abs(a.y - b.y) ) + 1;
  vector<Pixel> line( pixels );
  Interpolate( a, b, line );

  for (int i = 0; i < pixels; i++) {
    line[i].object = a.object;
    PixelShader(screen, lights, line[i], shadow);
  }
}

/* Setup the two arrays for start and end position of each row */
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels) {
  int V = vertexPixels.size();
  int minimumValue = numeric_limits<int>::max();
  int maximumValue = -numeric_limits<int>::max();

  // 1. Find max and min y-value of the polygon
  // and compute the number of rows it occupies.
  for(int i = 0; i < V; ++i) {
    if (vertexPixels[i].y > maximumValue) {
      maximumValue = vertexPixels[i].y;
    }
    if (vertexPixels[i].y < minimumValue) {
      minimumValue = vertexPixels[i].y;
    }
  }
  const int ROWS = (maximumValue - minimumValue) + 1;
  // std::cout << "Number of rows: " << ROWS << "\n";

  // 2. Resize leftPixels and rightPixels
  // so that they have an element for each row.
  leftPixels.resize(ROWS);
  rightPixels.resize(ROWS);

  // 3. Initialize the x-coordinates in leftPixels
  // to some really large value and the x-coordinates
  // in rightPixels to some really small value.
  for (int i = 0; i < ROWS; ++i) {
    leftPixels[i].x = numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }

  // 4. Loop through all edges of the polygon and use
  // linear interpolation to find the x-coordinate for
  // each row it occupies. Update the corresponding
  // values in rightPixels and leftPixels.
  for (int i = 0; i < V; ++i) {
    int j = (i+1)%V;
    int dx = abs(vertexPixels[i].x - vertexPixels[j].x);
    int dy = abs(vertexPixels[i].y - vertexPixels[j].y);
    int pixels = glm::max(dx, dy) + 1 ;
    vector<Pixel> result(pixels);
    Interpolate(vertexPixels[i], vertexPixels[j], result);

    for (int n = 0; n < pixels; ++n) {
      if (result[n].y >= minimumValue) {
        int pos = result[n].y - minimumValue;
        if (result[n].x < leftPixels[pos].x) {
          leftPixels[pos] = result[n];
        }
        if (result[n].x > rightPixels[pos].x) {
          rightPixels[pos] = result[n];
        }
      }
    }
  }
}

void DrawRows(screen* screen, const vector<Light> lights, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 shadow){
  for (int i = 0; i < leftPixels.size(); i++){
    DrawLineSDL(screen, lights, leftPixels[i], rightPixels[i], shadow);
  }
}

void DrawPolygon(screen* screen, const vector<Vertex>& vertices, const vector<Light> lights )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  vec3 shadow = vec3(1, 1, 1);
  for( int i=0; i<V; ++i ) {
    VertexShader( vertices[i], vertexPixels[i] );
    int x = vertexPixels[i].x;
    int y = vertexPixels[i].y;
    if (vertexPixels[i].zinv > shadowMap[y][x]) {
      shadow = vec3(0, 0, 0);
    }
  }
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  for (int i=0; i < leftPixels.size(); i++) {
    leftPixels[i].object = vertexPixels[0].object;
    rightPixels[i].object = vertexPixels[0].object;
  }
  DrawRows(screen, lights, leftPixels, rightPixels, shadow);
}

void getLensFlare(vector<vec2>& positions, vector<float>& scales, const vector<Light> lights){
  vec4 newLightPos = vec4(0, 0, 0, 0);
  for (size_t i = 0; i < lights.size(); i++) {
    newLightPos += lights[i].lightPos;
  }
  newLightPos /= lights.size();
  int lightX = focalLength * (newLightPos.x / newLightPos.z) + SCREEN_WIDTH / 2;
  int lightY = focalLength * (newLightPos.y / newLightPos.z) + SCREEN_HEIGHT / 2;
  vec2 light = vec2(lightX, lightY);
  vec2 centre = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
  vec2 A = centre - light;

  for (int i = 0; i < 5; i++){
    vec2 centrePoint = A * float((i+1));
    positions[i] = centrePoint;
    float scale = rand() % 2 + 1;
    scales[i] = scale/100;
  }
}

void changeLensFlare(vector<vec2>& positions, vector<float>& scales, const vector<Light> lights){
  vec4 newLightPos = vec4(0, 0, 0, 0);
  for (size_t i = 0; i < lights.size(); i++) {
    newLightPos += lights[i].lightPos;
  }
  newLightPos /= lights.size();
  int lightX = focalLength * (newLightPos.x / newLightPos.z) + SCREEN_WIDTH / 2;
  int lightY = focalLength * (newLightPos.y / newLightPos.z) + SCREEN_HEIGHT / 2;
  vec2 light = vec2(lightX, lightY);
  vec2 centre = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
  vec2 A = centre - light;

  for (int i = 0; i < positions.size(); i++){
    positions[i] = A * float((i+1));
    // scales[i] = rand() % 2 + int(fabs(cameraPos.z)) - 1;
    // cout << scales[i] << endl;
  }
}

void changeLights(float dx, float dz, vector<Light>& lights){
  for (int i = 0; i < lights.size(); i++){
    lights[i].lightPos.x += dx;
    lights[i].lightPos.z += dz;
  }
}

void rotateLights(mat4 M, vector<Light>& lights){
  for (int i = 0; i < lights.size(); i++){
    lights[i].lightPos = M * lights[i].lightPos;
  }
}

/*Place updates of parameters here*/
bool Update(vector<Light>& lights)
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  // std::cout << "Render time: " << dt << " ms." << std::endl;
  mat4 M;

    SDL_Event e;
    while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
      {
        return false;
      }
      else if (e.type == SDL_KEYDOWN)
      {
        int key_code = e.key.keysym.sym;
        switch(key_code)
        {
          case SDLK_UP:
            /* Move camera forwards */
            cameraPos.z += 0.1f;
            changeLights(0.f, -0.1f, lights);
            break;
          case SDLK_DOWN:
            /* Move camera backwards */
            cameraPos.z -= 0.1f;
            changeLights(0.f, 0.1f, lights);
            break;
          case SDLK_LEFT:
            /* Move camera left */
            cameraPos.x -= 0.1f;
            changeLights(0.1f, 0.f, lights);
            break;
          case SDLK_RIGHT:
            /* Move camera right */
            cameraPos.x += 0.1f;
            changeLights(-0.1f, 0.f, lights);
            break;
          // case SDLK_1:
          //   lights[0].lightPos.x -= 0.1f;
          //   break;
          // case SDLK_2:
          //   lights[0].lightPos.x += 0.1f;
          //   break;
          // case SDLK_3:
          //   lights[0].lightPos.y -= 0.1f;
          //   break;
          // case SDLK_4:
          //   lights[0].lightPos.y += 0.1f;
          //   break;
          // case SDLK_5:
          //   lights[0].lightPos.z -= 0.1f;
          //   break;
          // case SDLK_6:
          //   lights[0].lightPos.z += 0.1f;
          //   break;
          case SDLK_w:
            xaw -= 0.1f;
            TransformationMatrix(M);
            rotateLights(glm::inverse(M), lights);
            break;
          case SDLK_s:
            xaw += 0.1f;
            TransformationMatrix(M);
            rotateLights(glm::inverse(M), lights);
            break;
          case SDLK_a:
            yaw += 0.1f;
            TransformationMatrix(M);
            rotateLights(glm::inverse(M), lights);
            break;
          case SDLK_d:
            yaw -= 0.1f;
            TransformationMatrix(M);
            rotateLights(glm::inverse(M), lights);
            break;
          case SDLK_ESCAPE:
            /* Move camera quit */
            return false;
        }
      }
    }

  return true;
}

void TransformationMatrix(mat4& M) {
  mat4 C( 0.f, 0.f, 0.f, cameraPos.x,
          0.f, 0.f, 0.f, cameraPos.y,
          0.f, 0.f, 0.f, cameraPos.z,
          0.f, 0.f, 0.f, 1.f);
  mat4 NC( 0.f, 0.f, 0.f, -cameraPos.x,
          0.f, 0.f, 0.f, -cameraPos.y,
          0.f, 0.f, 0.f, -cameraPos.z,
          0.f, 0.f, 0.f, 1.f);

  mat4 R ( cos(yaw)*cos(zaw), -cos(yaw)*sin(zaw) + sin(xaw)*sin(yaw)*cos(zaw), sin(xaw)*sin(zaw) + cos(xaw)*sin(yaw)*cos(zaw), 0,
      cos(yaw)*sin(zaw), cos(xaw)*cos(zaw) + sin(xaw)*sin(yaw)*sin(zaw), -sin(xaw)*cos(zaw) + cos(xaw)*sin(yaw)*sin(zaw), 0,
      -sin(yaw)        , sin(xaw)*cos(yaw)                              , cos(xaw)*cos(yaw)                             , 0,
      0                , 0                                              , 0                                             , 1);

  M = R;
}
