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

bool Update();
void Draw(screen* screen, vector<Triangle> triangles, int lastIndex );
void VertexShader( const Vertex& vertices, Pixel& projPos );
void VertexShaderLight( const Vertex& vertex, Pixel& projPos );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 shadow );
void DrawPolygonEdges( screen* screen, const vector<Vertex>& vertices );
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void ComputeShadows( const vector<Vertex> vertices );
void TransformationMatrix( glm::mat4x4& M );
void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 shadow);
void DrawPolygon(screen* screen, const vector<Vertex>& vertices );
void PixelShader( screen* screen, const Pixel& p, vec3 shadow );
vec2 generateLensFlare(float scaleFactor);
void getLensFlare(vector<vec2>& positions, vector<float>& scales);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);
  int lightsStartIndex = triangles.size() - 1;
  LoadLightModel(triangles);
  vector<vec4> lightPositions(5);

  for (size_t i = 0; i < lights.size(); i++) {
    int randIndex = rand() % triangles.size();
    /* Correct the light's position wrt our camera */
    lightPositions.push_back(triangles[randIndex].v0 - cameraPos);
  }

  defineLights(lightPositions, lights);

  vector<vec2> randomPositions(3);
  vector<float> randomScales(3);
  getLensFlare(randomPositions, randomScales);
  while ( Update())
  {
      int lastIndex = triangles.size();
      if (prevCameraPos != cameraPos){
        prevCameraPos = cameraPos;
        randomPositions.clear();
        randomScales.clear();
        getLensFlare(randomPositions, randomScales);
      }
      for (int i = 0; i < 3; i++) {
        LoadApertureHexagon(triangles, randomPositions[i], randomScales[i]);
      }

      Draw(screen, triangles, lastIndex);
      triangles.erase(triangles.begin() + lastIndex, triangles.end());
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle> triangles, int lastIndex )
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  /* Clear depth and shadow buffer */
  for (int y = 0; y < SCREEN_HEIGHT; ++y) {
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
      depthBuffer[y][x] = 0;
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
      for(int v = 0; v < 3; ++v) {
          if (i < lastIndex) vertices[v].position = M * vertices[v].position;
          vertices[v].material = basicShader;
      }
      DrawPolygon( screen, vertices );
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
}


void PixelShader( screen* screen, const Pixel& pixel, vec3 shadow ) {
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

      R += diffuse + p * lights[i].indirectLightPowerPerArea;
    }

    if ( pixel.zinv > depthBuffer[y][x] ) {
      depthBuffer[y][x] = pixel.zinv;
      PutPixelSDL( screen, x, y, R);
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

void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 shadow ) {
  int pixels = glm::max( abs(a.x - b.x), abs(a.y - b.y) ) + 1;
  vector<Pixel> line( pixels );
  Interpolate( a, b, line );

  for (int i = 0; i < pixels; i++) {
    PixelShader(screen, line[i], shadow);
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

void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 shadow){
  for (int i = 0; i < leftPixels.size(); i++){
    DrawLineSDL(screen, leftPixels[i], rightPixels[i], shadow);
  }
}

void DrawPolygon(screen* screen, const vector<Vertex>& vertices )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  vec3 shadow = vec3(1, 1, 1);
  for( int i=0; i<V; ++i ){
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
  DrawRows(screen, leftPixels, rightPixels, shadow);
}

vec2 generateLensFlare(float scaleFactor) {
  vec4 newLightPos = vec4(-0.7, -0.1f, -0.7f, 1.0f);
  cout << newLightPos.x << " " << newLightPos.y << endl;
  int lightX = focalLength * (newLightPos.x / newLightPos.z) + SCREEN_WIDTH / 2;
  int lightY = focalLength * (newLightPos.y / newLightPos.z) + SCREEN_HEIGHT / 2;
  vec2 light = vec2(lightX, lightY);
  vec2 centre = vec2(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
  vec2 A = light - centre;
  vec2 drawPos = A * (scaleFactor + 1);

  return drawPos;
}

void getLensFlare(vector<vec2>& positions, vector<float>& scales){
  int xDiff = focalLength * (cameraPos.x / cameraPos.z) + SCREEN_WIDTH / 2;
  int yDiff = focalLength * (cameraPos.y / cameraPos.z) + SCREEN_HEIGHT / 2;
  vec2 diff = vec2(xDiff, yDiff);
  for (int i = 0; i < 3; i++){
    vec2 centrePoint = generateLensFlare((i+1)*10) + diff;
    positions[i] = centrePoint;
    float scale = rand() % 2 + 1;
    scales[i] = scale/100;
  }
}

/*Place updates of parameters here*/
bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  // std::cout << "Render time: " << dt << " ms." << std::endl;


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
            break;
          case SDLK_DOWN:
            /* Move camera backwards */
            cameraPos.z -= 0.1f;
            break;
          case SDLK_LEFT:
            /* Move camera left */
            cameraPos.x -= 0.1f;
            break;
          case SDLK_RIGHT:
            /* Move camera right */
            cameraPos.x += 0.1f;
            break;
          case SDLK_1:
            lights[0].lightPos.x -= 0.1f;
            break;
          case SDLK_2:
            lights[0].lightPos.x += 0.1f;
            break;
          case SDLK_3:
            lights[0].lightPos.y -= 0.1f;
            break;
          case SDLK_4:
            lights[0].lightPos.y += 0.1f;
            break;
          case SDLK_5:
            lights[0].lightPos.z -= 0.1f;
            break;
          case SDLK_6:
            lights[0].lightPos.z += 0.1f;
            break;
          case SDLK_w:
            xaw -= 0.1f;
            break;
          case SDLK_s:
            xaw += 0.1f;
            break;
          case SDLK_a:
            yaw += 0.1f;
            break;
          case SDLK_d:
            yaw -= 0.1f;
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
