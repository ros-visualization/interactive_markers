#ifndef OGRE_TOOLS_FPS_CAMERA_H_
#define OGRE_TOOLS_FPS_CAMERA_H_

#include "camera_base.h"

namespace Ogre
{
  class Camera;
  class SceneNode;
  class SceneManager;
}

namespace ogre_tools
{

class FPSCamera : public CameraBase
{
public:
  FPSCamera( Ogre::SceneManager* sceneManager );
  virtual ~FPSCamera();

  virtual void yaw( float angle );
  virtual void pitch( float angle );
  virtual void roll( float angle );
  virtual void setOrientation( float x, float y, float z, float w );
  virtual void setPosition( float x, float y, float z );
  virtual void setFrom( CameraBase* camera );

  virtual Ogre::Vector3 getPosition();
  virtual Ogre::Quaternion getOrientation();

  /// Move the camera relative to its forward axis
  virtual void move( float x, float y, float z );

  virtual void mouseLeftDrag( int diffX, int diffY );
  virtual void mouseMiddleDrag( int diffX, int diffY );
  virtual void mouseRightDrag( int diffX, int diffY );
  virtual void scrollWheel( int diff );

protected:
  void update();
  void normalizePitch();
  void normalizeYaw();

  float pitch_;
  float yaw_;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_FPS_CAMERA_H_*/
