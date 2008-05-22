#include <SDL.h>
#include <cstdio>
#include <unistd.h>
#include <string.h>
#include "cloud_viewer/cloud_viewer.h"

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("please give the cloudfile as the first parameter\n");
    return 0;
  }

  printf("opening [%s]\n", argv[1]);
  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    printf("couldn't open [%s]\n", argv[1]);
    return -1;
  }
  CloudViewer cloud_viewer;
  int line_num = 1;
  while (!feof(f))
  {
    double x, y, z, r, g, b;
    if (6 != fscanf(f, "%lf %lf %lf %lf %lf %lf\n", &x, &y, &z, &r, &g, &b))
    {
      printf("bad syntax on line %d\n", line_num);
      break;
    }
    //y *= -1; // convert from right-hand coordinate system
    if (line_num == 1)
    {
      printf("%f %f %f\n", x, y, z);
    }
    cloud_viewer.add_point(x, y, z, 255*r, 255*g, 255*b);
    line_num++;
  }
  printf("read %d points\n", line_num);

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "video init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
//  const int w = 640, h = 480;
  const int w = 1280, h = 960;
  if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)
  {
    fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
    return 2;
  }

  cloud_viewer.set_opengl_params(w,h);
  cloud_viewer.set_look_tgt(0, 0.5, 1.0);
  cloud_viewer.render();
  SDL_GL_SwapBuffers();
/*
  for (int i = 0; i < 10000; i++)
  {
    cloud_viewer.add_point(rand() % 10, rand() % 10, rand() % 10, 255, 255, 255);
  }
*/

  bool done = false;
  while(!done)
  {
    usleep(1000);
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_MOUSEMOTION:
          cloud_viewer.mouse_motion(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel);
          cloud_viewer.render();
          SDL_GL_SwapBuffers();
          break;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
        {
          int button = -1;
          switch(event.button.button)
          {
            case SDL_BUTTON_LEFT: button = 0; break;
            case SDL_BUTTON_MIDDLE: button = 1; break;
            case SDL_BUTTON_RIGHT: button = 2; break;
          }
          cloud_viewer.mouse_button(event.button.x, event.button.y,
            button, (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
          break;
        }
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else
            cloud_viewer.keypress(event.key.keysym.sym);
          cloud_viewer.render();
          SDL_GL_SwapBuffers();
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  SDL_Quit();

  return 0;
}
