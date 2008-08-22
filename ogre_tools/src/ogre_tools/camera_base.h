#ifndef OGRE_TOOLS_CAMERA_BASE_H_
#define OGRE_TOOLS_CAMERA_BASE_H_

#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace Ogre
{
  class Camera;
  class SceneNode;
  class SceneManager;
}

namespace ogre_tools
{

class CameraBase
{
public:
  CameraBase( Ogre::SceneManager* sceneManager );

  virtual ~CameraBase();

  virtual void Yaw( float angle ) = 0;
  virtual void Pitch( float angle ) = 0;
  virtual void Roll( float angle ) = 0;
  virtual void SetOrientation( float x, float y, float z, float w ) = 0;
  virtual void SetPosition( float x, float y, float z ) = 0;
  virtual void SetFrom( CameraBase* camera ) = 0;

  void SetPosition( const Ogre::Vector3& position );
  void SetOrientation( const Ogre::Quaternion& orientation );

  virtual Ogre::Vector3 GetPosition() = 0;
  virtual Ogre::Quaternion GetOrientation() = 0;

  /// Move the camera relative to its forward axis
  virtual void Move( float x, float y, float z ) = 0;

  Ogre::Camera* GetOgreCamera() { return camera_; }

  virtual void MouseLeftDrag( int diffX, int diffY ) = 0;
  virtual void MouseMiddleDrag( int diffX, int diffY ) = 0;
  virtual void MouseRightDrag( int diffX, int diffY ) = 0;
  virtual void ScrollWheel( int diff ) = 0;

protected:
  Ogre::Camera* camera_;
  Ogre::SceneManager* scene_manager_;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_CAMERA_BASE_H_*/
