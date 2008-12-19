/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "ros/node.h"
#include "std_msgs/PointCloud.h"
#include "std_msgs/Empty.h"
#include "cloud_viewer/cloud_viewer.h"
#include <SDL/SDL.h>
#include "math.h"
#include "sdlgl/sdlgl.h"
#include "rosthread/mutex.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#include <vector>
using namespace std;
struct cloud_point {
public:
  float x;
  float y;
  float z;
  float i;

  cloud_point(float _x, float _y, float _z, float _i) : x(_x), y(_y), z(_z), i(_i) {}
};

class Cloud_Node : public ros::node, public ros::SDLGL
{
public:
  std_msgs::PointCloud cloud;
  std_msgs::Empty shutter;
  CloudViewer cloud_viewer;

  int level;
  int spread;

  vector<cloud_point> buf[2];
  int buf_read_ind;
  int buf_use_ind;

  char dir_name[256];
  int cloud_cnt;

  bool made_dir;

  ros::thread::mutex cloud_mutex;

  Cloud_Node() : ros::node("cloud_viewer"), level(20), spread(400), buf_read_ind(0), buf_use_ind(1), cloud_cnt(0), made_dir(false)
  {
    subscribe("cloud", cloud, &Cloud_Node::cloud_callback);
    subscribe("shutter", shutter, &Cloud_Node::shutter_callback);

    if (!init_gui(1024, 768))
      self_destruct();

    cloud_viewer.set_opengl_params(1024,768);

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    sprintf(dir_name, "clouds_%.2d%.2d%.2d_%.2d%.2d%.2d", timeinfo->tm_mon + 1, timeinfo->tm_mday,timeinfo->tm_year - 100,timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

  }

  virtual ~Cloud_Node()
  {
  }

  virtual void render()
  {
    cloud_mutex.lock();     // I've commandeered the cloud mutex for my own nefarious purposes
    cloud_viewer.render();
    cloud_mutex.unlock();
    SDL_GL_SwapBuffers();
  }

  virtual void mouse_motion(int x, int y, int dx, int dy, int buttons)
  {
    cloud_mutex.lock();
    cloud_viewer.mouse_motion(x, y, dx, dy);
    cloud_mutex.unlock();
    request_render();
  }

  virtual void mouse_button(int x, int y, int button, bool is_down)
  {
    switch(button) {
    case SDL_BUTTON_LEFT: button = 0; break;
    case SDL_BUTTON_MIDDLE: button = 1; break;
    case SDL_BUTTON_RIGHT: button = 2; break;
    }
    cloud_mutex.lock();
    cloud_viewer.mouse_button(x, y, button, is_down);
    cloud_mutex.unlock();
    request_render();
  }

  virtual void keypress(char c, uint16_t u, SDLMod mod)
  {

    if (c == 91 || c == 93)
    {
      if (c == 91 && mod == 0)
	level -= 5;
      else if (c == 93 && mod == 0)
	level += 5;
      else if (c == 91 && mod == 1)
	spread -= 5;
      else if (c == 93 && mod == 1)
	spread += 5;

      printf("Level: %d, Spread: %d\n",level,spread);
      refill_cloud();
    }

    if (c == SDLK_RETURN) {
      if (mod == 1)
	save_VRML();
      else
	save_dat();
    }

    cloud_mutex.lock();
    cloud_viewer.keypress(c);
    cloud_mutex.unlock();
    request_render();
  }

  void cloud_callback()
  {
    int intensity_ind = -1;

    for (int i = 0; i < cloud.get_chan_size(); i++)
      if (cloud.chan[i].name == string("intensities"))
	intensity_ind = i;

    double intensity;

    for (int i = 0; i < cloud.get_pts_size(); i++)
    {
      intensity = 4000;
      if (intensity_ind != -1)
	intensity = cloud.chan[intensity_ind].vals[i];

      buf[buf_read_ind].push_back(cloud_point(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z, intensity));

    }

    /*
    int intensity;

    for (int i = 0; i < cloud.get_pts_size(); i++)
    {
      if (intensity_ind < 0)
      {
	intensity = 0;
      } else {
	intensity = level + cloud.chan[intensity_ind].vals[i] / spread;
	if (intensity > 255)
	  intensity = 255;
	else if (intensity < 0)
	  intensity = 0;
      }

      cloud_viewer.add_point(-cloud.pts[i].y, -cloud.pts[i].z, cloud.pts[i].x,
			       intensity, intensity, intensity);
    }
    */

  }

  void shutter_callback()
  {

    buf_use_ind =  buf_read_ind;
    buf_read_ind = !buf_read_ind;

    buf[buf_read_ind].clear();

    refill_cloud();
  }

  void refill_cloud()
  {
    cloud_mutex.lock();

    cloud_viewer.clear_cloud();

    double val = 0.0;
    double valsq = 0.0;
    int cnt = 0;

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      val += buf[buf_use_ind][i].i;
      valsq += pow((double)buf[buf_use_ind][i].i, 2.0);
      cnt++;
    }
    double mean = val / cnt;
    double std = sqrt( (valsq - cnt*pow(mean, 2.0))/(cnt - 1));

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      float intensity = level + spread * (buf[buf_use_ind][i].i - mean) / (2*std);

      if (intensity > 255)
	intensity = 255;
      else if (intensity < 0)
	intensity = 0;

      cloud_viewer.add_point(-buf[buf_use_ind][i].y, -buf[buf_use_ind][i].z, buf[buf_use_ind][i].x,
			     intensity, intensity, intensity);
    }

    cloud_mutex.unlock();

    request_render();
  }

  void save_VRML() {

    printf("Trying to save..\n");

    if (!made_dir) {
      if (mkdir(dir_name, 0755)) {
	printf("Failed to make directory: %s\n", dir_name);
	return;
      } else {
	made_dir = true;
      }
    }

    std::ostringstream oss;
    oss << dir_name << "/Cloud" << cloud_cnt++ << ".wrl";
    ofstream out(oss.str().c_str());

    out.setf(ios::fixed, ios::floatfield);
    out.setf(ios::showpoint);
    out.precision(4);

    double val = 0.0;
    double valsq = 0.0;
    int cnt = 0;

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      val += buf[buf_use_ind][i].i;
      valsq += pow((double)buf[buf_use_ind][i].i, 2.0);
      cnt++;
    }
    double mean = val / cnt;
    double std = sqrt( (valsq - cnt*pow(mean, 2.0))/(cnt - 1));

    out << "#VRML V2.0 utf8" << endl
	<< "DEF InitView Viewpoint" << endl
	<< "{" << endl
	<< "position -2 0 0" << endl
	<< "orientation 0 1 0 -1.57079633" << endl
	<< "" << endl
	<< "}DEF World Transform " << endl
	<< "{" << endl
	<< "translation 0 0 0" << endl
	<< "children  [" << endl
	<< "DEF Scene Transform" << endl
	<< "{" << endl
	<< "translation 0 0 0" << endl
	<< "children [" << endl
	<< "Shape" << endl
	<< "{" << endl
	<< "geometry PointSet" << endl
	<< "{" << endl
	<< "coord Coordinate" << endl
	<< "{" << endl
	<< "point[" << endl;

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      out << buf[buf_use_ind][i].x << " " << buf[buf_use_ind][i].z << " " << -buf[buf_use_ind][i].y << ", " << endl;
    }

    out << "]"
	<< "}" << endl
	<< "color Color" << endl
	<< "{" << endl
	<< "color[" << endl;

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      float intensity = (level + spread * (buf[buf_use_ind][i].i - mean) / (2*std)) / 255;

      if (intensity > 255)
	intensity = 255;
      else if (intensity < 0)
	intensity = 0;

      out << intensity << " " << intensity << " " << intensity << "," << endl;
    }

    out << "]"
	<< "}" << endl
	<< "}" << endl
	<< "}" << endl
	<< "" << endl
	<< "]" << endl
	<< "} # end of scene transform" << endl
	<< "]" << endl
	<< "} # end of world transform" << endl;

  }


  void save_dat() {

    printf("Trying to save..\n");

    if (!made_dir) {
      if (mkdir(dir_name, 0755)) {
	printf("Failed to make directory: %s\n", dir_name);
	return;
      } else {
	made_dir = true;
      }
    }

    std::ostringstream oss;
    oss << dir_name << "/Cloud" << cloud_cnt++ << ".dat";
    ofstream out(oss.str().c_str());

    out.setf(ios::fixed, ios::floatfield);
    out.setf(ios::showpoint);
    out.precision(5);

    out << "#Point Cloud" << endl;
    out << "#Format: X [meters], Y [meters], Z [meters], Intensity" << endl;
    out << "#Points: " << buf[buf_use_ind].size() << endl;

    for (int i = 0; i < buf[buf_use_ind].size(); i++) {
      out << buf[buf_use_ind][i].x << " "
	  << buf[buf_use_ind][i].y << " "
	  << buf[buf_use_ind][i].z << " "
	  << buf[buf_use_ind][i].i << endl;
    }

    out.close();

  }


};


int main(int argc, char **argv)
{
  ros::init(argc, argv);

  atexit(SDL_Quit);

  Cloud_Node cn;

  cn.main_loop();

  ros::fini();
  return 0;
}

