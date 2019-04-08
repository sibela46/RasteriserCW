#include <iostream>
#include<fstream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include "TestModelH.h"
#include "Lightbulb.h"
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
void Draw(screen* screen, vector<Triangle> triangles );
void VertexShader( const Vertex& vertices, Pixel& projPos );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( screen* screen, const vector<Vertex>& vertices, vec3 color );
void ComputePolygonRows( const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void TransformationMatrix( glm::mat4x4& M );
void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color);
void DrawPolygon(screen* screen, const vector<Vertex>& vertices, vec3 color  );
void PixelShader( screen* screen, const Pixel& p );

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);
  // LoadBunnyModel(triangles);
  float offset = lightPos.y;
  // LoadApertureHexagon(triangles, 0.05, offset);
  LoadLightModel(triangles);

  /* Correct the light's position wrt our camera */
  lightPos = lightPos - cameraPos;

  while ( Update())
    {
      Draw(screen, triangles);
      int lightX = focalLength * (lightPos.x / lightPos.z) + SCREEN_WIDTH / 2;
      int lightY = focalLength * (lightPos.y / lightPos.z) + SCREEN_HEIGHT / 2;
      PutPixelSDL(screen, lightX, lightY, vec3(1, 0, 0));
      // DrawSphere(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, vector<Triangle> triangles )
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  /* Clear depth buffer */
  for (int y = 0; y < SCREEN_HEIGHT; ++y) {
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
      depthBuffer[y][x] = 0;
    }
  }
  /* Setup rotation matrix */
  mat4 M;
  TransformationMatrix(M);
  for( uint32_t i=0; i < triangles.size(); ++i ) {
      vector<Vertex> vertices(3);
      vertices[0].position = triangles[i].v0;
      vertices[1].position = triangles[i].v1;
      vertices[2].position = triangles[i].v2;
      vec3 color = triangles[i].color;
      currentNormal = triangles[i].normal;
      currentReflectance = color;
      for(int v = 0; v < 3; ++v) {
          vertices[v].position = M * vertices[v].position;
          vertices[v].material = basicShader;
      }
      DrawPolygon( screen, vertices, color);
  }
}

void VertexShader( const Vertex& vertex, Pixel& projPos ) {
  vec4 temp = vertex.position - cameraPos; // * cam_rotation
  projPos.x = focalLength * (temp.x / temp.z) + SCREEN_WIDTH / 2;
  projPos.y = focalLength * (temp.y / temp.z) + SCREEN_HEIGHT / 2;
  projPos.zinv = focalLength * (1 / (temp.z));
  projPos.pos3d = temp;
}

void PixelShader( screen* screen, const Pixel& pixel ) {
    int x = pixel.x;
    int y = pixel.y;

    /* Compute illumination */
    float r = glm::distance(pixel.pos3d, lightPos);
    float A = 4 * M_PI * r * r;
    vec4 r_hat = glm::normalize(lightPos - pixel.pos3d);
    vec4 n = currentNormal;
    vec3 p = currentReflectance;
    vec3 P = lightPower / A;
    vec3 D = P * max(glm::dot(n, r_hat), 0.0f);
    vec3 diffuse = p * D;

    /* Ambient */
    vec3 ambient = p * basicShader.ambient;

    /* Specular */
    vec4 viewDir = glm::normalize(cameraPos - pixel.pos3d);
    vec4 reflectDir = reflect(r_hat, currentNormal);
    float spec = pow(max(glm::dot(viewDir, reflectDir), 0.0f), basicShader.shininess);
    vec3 specular = 0.5f * p * (spec * basicShader.specular);

    // vec3 R = p * D + indirectLightPowerPerArea;

    vec3 R = diffuse + p * indirectLightPowerPerArea;

    if ( pixel.zinv > depthBuffer[y][x] ) {
      depthBuffer[y][x] = pixel.zinv;
      PutPixelSDL( screen, x, y, R);
    }
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ) {
  int N = result.size();
  float stepX = (b.x-a.x) / float(max(N-1,1));
  float stepY = (b.y-a.y) / float(max(N-1,1));
  float stepZ = (b.zinv-a.zinv) / float(max(N-1,1));
  vec4 stepI = (b.pos3d*b.zinv-a.pos3d*a.zinv) / float(max(N-1,1));
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

void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 color ) {
  int pixels = glm::max( abs(a.x - b.x), abs(a.y - b.y) ) + 1;
  vector<Pixel> line( pixels );
  Interpolate( a, b, line );

  for (int i = 0; i < pixels; i++) {
    PixelShader(screen, line[i]);
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

void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color){
  for (int i = 0; i < leftPixels.size(); i++){
    DrawLineSDL(screen, leftPixels[i], rightPixels[i], color);
  }
}

void DrawPolygon(screen* screen, const vector<Vertex>& vertices, vec3 color )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i ){
    VertexShader( vertices[i], vertexPixels[i] );
  }
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawRows(screen, leftPixels, rightPixels, color);
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
            lightPos.x -= 0.1f;
            break;
          case SDLK_2:
            lightPos.x += 0.1f;
            break;
          case SDLK_3:
            lightPos.y -= 0.1f;
            break;
          case SDLK_4:
            lightPos.y += 0.1f;
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

// void DrawPolygonEdges( screen* screen, const vector<Vertex>& vertices, vec3 color ){
  //   int V = vertices.size();
  //   // Transform each vertex from 3D world position to 2D image position:
  //   vector<Pixel> projectedVertices( V );
  //   for( int i = 0; i < V; ++i ){
    //     VertexShader( vertices[i], projectedVertices[i] );
    //   }
    //   // Loop over all vertices and draw the edge from it to the next vertex:
    //   for( int i = 0; i < V; ++i ){
      //     int j = (i+1)%V; // The next vertex
      //     DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
      //   }
      // }
