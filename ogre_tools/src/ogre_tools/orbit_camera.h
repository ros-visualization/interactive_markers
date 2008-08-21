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
  OrbitCamera( Ogre::SceneManager* sceneManager );
  virtual ~OrbitCamera();

  void Zoom( float amount );
  void SetFocalPoint( const Ogre::Vector3& focalPoint );
  void SetFrom( CameraBase* camera );

  virtual void Yaw( float angle );
  virtual void Pitch( float angle );
  virtual void Roll( float angle );
  virtual void SetOrientation( float x, float y, float z, float w );
  virtual void SetPosition( float x, float y, float z );

  virtual Ogre::Vector3 GetPosition();
  virtual Ogre::Quaternion GetOrientation();

  virtual void Move( float x, float y, float z );

  virtual void MouseLeftDrag( int diffX, int diffY );
  virtual void MouseMiddleDrag( int diffX, int diffY );
  virtual void MouseRightDrag( int diffX, int diffY );
  virtual void ScrollWheel( int diff );

private:
  void Update();
  void CalculatePitchYawFromPosition( const Ogre::Vector3& position );
  void NormalizePitch();
  void NormalizeYaw();

  Ogre::Vector3 m_FocalPoint;
  float m_Yaw;
  float m_Pitch;
  float m_Distance;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_ORBIT_CAMERA_H_*/
