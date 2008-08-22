#ifndef OGRE_TOOLS_ORBIT_CAMERA_H_
#define OGRE_TOOLS_ORBIT_CAMERA_H_

#include "camera_base.h"
#include <OgreVector3.h>

namespace Ogre
{
  class Camera;
  class SceneNode;
  class SceneManager;
}

namespace ogre_tools
{

class OrbitCamera : public CameraBase
{
public:
  OrbitCamera( Ogre::SceneManager* scene_manager );
  virtual ~OrbitCamera();

  void zoom( float amount );
  void setFocalPoint( const Ogre::Vector3& focal_point );
  void setFrom( CameraBase* camera );

  virtual void yaw( float angle );
  virtual void pitch( float angle );
  virtual void roll( float angle );
  virtual void setOrientation( float x, float y, float z, float w );
  virtual void setPosition( float x, float y, float z );

  virtual Ogre::Vector3 getPosition();
  virtual Ogre::Quaternion getOrientation();

  virtual void move( float x, float y, float z );

  virtual void mouseLeftDrag( int diff_x, int diff_y );
  virtual void mouseMiddleDrag( int diff_x, int diff_y );
  virtual void mouseRightDrag( int diff_x, int diff_y );
  virtual void scrollWheel( int diff );

private:
  void update();
  void calculatePitchYawFromPosition( const Ogre::Vector3& position );
  void normalizePitch();
  void normalizeYaw();

  Ogre::Vector3 focal_point_;
  float yaw_;
  float pitch_;
  float distance_;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_ORBIT_CAMERA_H_*/
