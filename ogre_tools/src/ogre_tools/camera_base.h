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
  CameraBase( Ogre::SceneManager* scene_manager );

  virtual ~CameraBase();

  virtual void yaw( float angle ) = 0;
  virtual void pitch( float angle ) = 0;
  virtual void roll( float angle ) = 0;
  virtual void setOrientation( float x, float y, float z, float w ) = 0;
  virtual void setPosition( float x, float y, float z ) = 0;
  virtual void setFrom( CameraBase* camera ) = 0;

  void setPosition( const Ogre::Vector3& position );
  void setOrientation( const Ogre::Quaternion& orientation );

  virtual Ogre::Vector3 getPosition() = 0;
  virtual Ogre::Quaternion getOrientation() = 0;

  /// Move the camera relative to its forward axis
  virtual void move( float x, float y, float z ) = 0;

  Ogre::Camera* getOgreCamera() { return camera_; }

  virtual void mouseLeftDrag( int diff_x, int diff_y ) = 0;
  virtual void mouseMiddleDrag( int diff_x, int diff_y ) = 0;
  virtual void mouseRightDrag( int diff_x, int diff_y ) = 0;
  virtual void scrollWheel( int diff ) = 0;

protected:
  Ogre::Camera* camera_;
  Ogre::SceneManager* scene_manager_;
};

} // namespace ogre_tools

#endif /*OGRE_TOOLS_CAMERA_BASE_H_*/
