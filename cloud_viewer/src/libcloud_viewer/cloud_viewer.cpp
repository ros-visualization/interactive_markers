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

#define _USE_MATH_DEFINES
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "cloud_viewer/cloud_viewer.h"


CloudViewer::CloudViewer() : 
	cam_x(0), cam_y(0), cam_z(0),
	cam_azi(-M_PI/2), cam_ele(0), cam_rho(1),
	look_tgt_x(0), look_tgt_y(0), look_tgt_z(0),
	left_button_down(false), right_button_down(false),
  hide_axes(false), postrender_cb(NULL),
  state(UNINIT), points_list(0)
{
}

CloudViewer::~CloudViewer()
{
}

void CloudViewer::clear_cloud()
{
	points.clear();
}

void CloudViewer::add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b,
                            float *extra, uint32_t num_extra)
{
	points.push_back(CloudViewerPoint(x,y,z,r,g,b,extra,num_extra));
}

void CloudViewer::render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glScalef(1.0f, 1.0f, 1.0f);
	// convert camera from spherical coordinates to cartesian...
	cam_x = cam_rho * sinf((float)(M_PI/2) - cam_ele) * cosf(cam_azi) + look_tgt_x;
	cam_y = cam_rho * cosf((float)(M_PI/2) - cam_ele)                 + look_tgt_y;
	cam_z = cam_rho * sinf((float)(M_PI/2) - cam_ele) * sinf(cam_azi) + look_tgt_z;
	gluLookAt(cam_x, cam_y, cam_z, look_tgt_x, look_tgt_y, look_tgt_z, 0, -1, 0);

  if (!hide_axes)
  {
	  glPushMatrix();
  	glTranslatef(look_tgt_x, look_tgt_y, look_tgt_z);
  	const float axes_length = 0.1f;
  	glLineWidth(2);
  	glBegin(GL_LINES);
  		glColor3f(1,0.5,0.5);
  		glVertex3f(0,0,0);
  		glVertex3f(axes_length,0,0);
  		glColor3f(0.5,1,0.5);
  		glVertex3f(0,0,0);
  		glVertex3f(0,axes_length,0);
  		glColor3f(0.5,0.5,1);
  		glVertex3f(0,0,0);
    	glVertex3f(0,0,axes_length);
  	glEnd();
  	glLineWidth(1);
  	glPopMatrix();
	
    // draw a vector from the origin so we don't get lost
    glBegin(GL_LINES);
  		glColor3f(1,1,1);
  		glVertex3f(0,0,0);
  		glVertex3f(look_tgt_x, look_tgt_y, look_tgt_z);
  	glEnd();
  }

  //if (state == UNINIT)
  //{
    //points_list = glGenLists(1);
    //glNewList(points_list, GL_COMPILE_AND_EXECUTE);
	  glBegin(GL_POINTS);
  	for (size_t i = 0; i < points.size(); i++)
  	{
  		glColor3ub(points[i].r, points[i].g, points[i].b);
  		glVertex3f(points[i].x, points[i].y, points[i].z);
  	}
  	glEnd();
    //glEndList();
    //state = LIST_GENERATED;
 // }
  //else
  //  glCallList(points_list); // video ram is so cool

  if (postrender_cb)
    postrender_cb();
}

void CloudViewer::set_opengl_params(unsigned width, unsigned height)
{
	glClearColor(0.1f, 0.1f, 0.3f, 0.0f);
	glViewport(0, 0, (GLint)width, (GLint)height);
	glEnable(GL_DEPTH_TEST);
	double aspect_ratio = (double)width / height;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, aspect_ratio, 0.01, 50.0);
}

void CloudViewer::mouse_button(int x, int y, int button, bool is_down)
{
	if (button == 0)
		left_button_down = is_down;
	else if (button == 2)
		right_button_down = is_down;
}

void CloudViewer::mouse_motion(int x, int y, int dx, int dy)
{
	if (left_button_down)
	{
		cam_azi += 0.01f * dx;
		cam_ele += 0.01f * dy;
		// saturate cam_ele to prevent blowing through the singularity at cam_ele = +/- PI/2
		if (cam_ele > 1.5f)
			cam_ele = 1.5f;
		else if (cam_ele < -1.5f)
			cam_ele = -1.5f;
	}
	else if (right_button_down)
		cam_rho *= (1.0f + 0.01f * dy);
}

void CloudViewer::keypress(char c)
{
	switch(c)
	{
		case 'w': look_tgt_z += 0.05; break;
		case 'x': look_tgt_z -= 0.05; break;
		case 'a': look_tgt_x -= 0.05; break;
		case 'd': look_tgt_x += 0.05; break;
		case 'i': look_tgt_y -= 0.05; break;
		case 'k': look_tgt_y += 0.05; break;
    case 'h': hide_axes = !hide_axes; break;
	}
}

bool CloudViewer::write_file(const std::string &filename)
{
  FILE *f = fopen(filename.c_str(), "w");
  if (!f)
    return false;
  for (size_t i = 0; i < points.size(); i++)
  {
    fprintf(f, "%f %f %f %f %f %f ", 
            points[i].x,
            points[i].y,
            points[i].z,
            points[i].r / 255.0,
            points[i].g / 255.0,
            points[i].b / 255.0);
    for (size_t j = 0; j < points[i].num_extra; j++)
      fprintf(f, "%f ", points[i].extra[j]);
    fprintf(f, "\n");
  }
  fclose(f);
  return true;
}

