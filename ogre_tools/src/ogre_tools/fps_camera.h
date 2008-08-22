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

  virtual void Yaw( float angle );
  virtual void Pitch( float angle );
  virtual void Roll( float angle );
  virtual void SetOrientation( float x, float y, float z, float w );
  virtual void SetPosition( float x, float y, float z );
  virtual void SetFrom( CameraBase* camera );

  virtual Ogre::Vector3 GetPosition();
  virtual Ogre::Quaternion GetOrientation();

  /// Move the camera relative to its forward axis
  virtual void Move( float x, float y, float z );

  virtual void MouseLeftDrag( int diffX, int diffY );
  virtual void MouseMiddleDrag( int diffX, int diffY );
  virtual void MouseRightDrag( int diffX, int diffY );
  virtual void ScrollWheel( int diff );

protected:
  void Update();
  void NormalizePitch();
  void NormalizeYaw();

  float pitch_;
  float yaw_;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_FPS_CAMERA_H_*/
