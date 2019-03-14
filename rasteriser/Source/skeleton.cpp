#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;
using glm::ivec2;

SDL_Event event;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen, vector<Triangle> triangles );
void VertexShader( const vec4& vertices, ivec2& projPos );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices, vec3 color );
void ComputePolygonRows( const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels);
void TransformationMatrix( glm::mat4x4& M );
void DrawRows(screen* screen, const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels, vec3 color);
void DrawPolygon(screen* screen, const vector<vec4>& vertices, vec3 color  );

float xaw = 0.f;
float yaw = 0.f;
float zaw = 0.f;
vec4 cameraPos = vec4(0.f, 0.f, -2.5f, 1.f);
float focalLength = SCREEN_HEIGHT/2;

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  vector<Triangle> triangles;
  LoadTestModel(triangles);

  while ( Update())
    {
      Draw(screen, triangles);
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
  /* Setup rotation matrix */
  mat4 M;
  TransformationMatrix(M);
  for( uint32_t i=0; i < triangles.size(); ++i ) {
      vector<vec4> vertices(3);
      vertices[0] = triangles[i].v0;
      vertices[1] = triangles[i].v1;
      vertices[2] = triangles[i].v2;
      vec3 color = triangles[i].color;
      for(int v = 0; v < 3; ++v) {
          vertices[v] = M * vertices[v];
      }
      DrawPolygon( screen, vertices, color);
  }
}

void VertexShader( const vec4& vertex, ivec2& projPos ) {
  vec4 temp = vertex - cameraPos; // * cam_rotation
  projPos.x = focalLength * (temp.x / temp.z) + SCREEN_WIDTH / 2;
  projPos.y = focalLength * (temp.y / temp.z) + SCREEN_HEIGHT / 2;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result ) {
  int N = result.size();
  vec2 step = vec2(b-a) / float(max(N-1,1));
  vec2 current( a );
  for( int i=0; i<N; ++i ) {
    result[i] = current;
    current += step;
  }
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ) {
  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels );
  Interpolate( a, b, line );

  for (int i = 0; i < pixels; i++) {
    PutPixelSDL( screen, line[i].x, line[i].y, color );
  }
}

void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices, vec3 color ){
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i = 0; i < V; ++i ){
    VertexShader( vertices[i], projectedVertices[i] );
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i = 0; i < V; ++i ){
    int j = (i+1)%V; // The next vertex
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}

/* Setup the two arrays for start and end position of each row */
void ComputePolygonRows( const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels) {
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
    ivec2 delta = glm::abs(vertexPixels[i] - vertexPixels[j]);
    int pixels = glm::max(delta.x, delta.y) + 1 ;
    vector<ivec2> result(pixels);
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

void DrawRows(screen* screen, const vector<ivec2>& leftPixels, const vector<ivec2>& rightPixels, vec3 color){
  for (int i = 0; i < leftPixels.size(); i++){
    DrawLineSDL(screen, leftPixels[i], rightPixels[i], color);
  }
}

void DrawPolygon(screen* screen, const vector<vec4>& vertices, vec3 color )
{
  int V = vertices.size();
  vector<ivec2> vertexPixels( V );
  for( int i=0; i<V; ++i ){
    VertexShader( vertices[i], vertexPixels[i] );
  }
  vector<ivec2> leftPixels;
  vector<ivec2> rightPixels;
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
