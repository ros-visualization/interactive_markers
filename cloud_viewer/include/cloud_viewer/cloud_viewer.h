#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include <vector>
#include <string>

class CloudViewerPoint
{
public:
	float x, y, z;
	uint8_t r, g, b;
  /** extra data that this point carries along with it (e.g. row/col of an
      original image that this point was generated from) */
  float *extra;
  uint32_t num_extra;
	CloudViewerPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b, 
                   float *_extra = NULL, uint32_t _num_extra = 0)
  : x(x), y(y), z(z), r(r), g(g), b(b)
  {
    if (_num_extra > 0)
    {
      num_extra = _num_extra;
      extra = new float[num_extra];
      for (uint32_t i = 0; i < num_extra; i++)
        extra[i] = _extra[i];
    }
    else
      extra = NULL;
  }
  CloudViewerPoint(const CloudViewerPoint &ref)
  : x(ref.x), y(ref.y), z(ref.z), 
    r(ref.r), g(ref.g), b(ref.b),
    num_extra(ref.num_extra)
  {
    if (num_extra > 0)
    {
      extra = new float[num_extra];
      for (uint32_t i = 0; i < num_extra; i++)
        extra[i] = ref.extra[i];
    }
    else
      extra = NULL;
  }
  ~CloudViewerPoint()
  {
    if (extra)
      delete[] extra;
    extra = NULL;
  }
};

class CloudViewer
{
public:
	CloudViewer();
	~CloudViewer();

	void clear_cloud();
	void add_point(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b,
                 float *extra = 0, uint32_t num_extra = 0);
	void set_opengl_params(unsigned width, unsigned height);
	void render();
	void mouse_button(int x, int y, int button, bool is_down);
	void mouse_motion(int x, int y, int dx, int dy);
	void keypress(char c);
  void set_look_tgt(double x, double y, double z) 
  { look_tgt_x = x; look_tgt_y = y; look_tgt_z = z; }
  bool write_file(const std::string &filename);

private:
	std::vector<CloudViewerPoint> points;
	float cam_x, cam_y, cam_z, cam_azi, cam_ele, cam_rho;
	float look_tgt_x, look_tgt_y, look_tgt_z;
	bool left_button_down, right_button_down;
  bool hide_axes;
};

#endif

