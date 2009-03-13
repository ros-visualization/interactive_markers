///////////////////////////////////////////////////////////////////////////////
// This file provides a handy little cloud viewer
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.


#include <SDL.h>
#include <cstdio>
#include <unistd.h>
#include <string.h>
#include "cloud_viewer/cloud_viewer.h"
#include <ctime>

int main(int argc, char **argv)
{
  srand(time(NULL));
  double frac_to_show = 1;
  if (argc < 2)
  {
    printf("please give the cloudfile as the first parameter\n");
    return 0;
  }
  if (argc >= 3)
    frac_to_show = atof(argv[2]);

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
    if (line_num == 1)
    {
      printf("%f %f %f\n", x, y, z);
    }
    if (frac_to_show < 1.0)
    {
      if (rand() % 1000 <= (int)(frac_to_show * 1000))
        cloud_viewer.add_point(x, y, z, 255*r, 255*g, 255*b);
    }
    else
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
  const int w = 1280, h = 960;
  if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)
  {
    fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
    return 2;
  }

  cloud_viewer.set_opengl_params(w,h);
  cloud_viewer.set_look_tgt(0, 0, 0);
  cloud_viewer.render();
  SDL_GL_SwapBuffers();

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
          {
            uint32_t mod = 0;
            if (event.key.keysym.sym & KMOD_SHIFT)
              mod |= CloudViewer::MOD_SHIFT;
            cloud_viewer.keypress(event.key.keysym.sym, mod);
          }
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
