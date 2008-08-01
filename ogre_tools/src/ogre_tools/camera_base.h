#ifndef OGRE_TOOLS_CAMERA_BASE_H_
#define OGRE_TOOLS_CAMERA_BASE_H_

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
  
  /// Move the camera relative to its forward axis
  virtual void Move( float x, float y, float z ) = 0;
  virtual void SetPosition( float x, float y, float z ) = 0;
  
  Ogre::Camera* GetOgreCamera() { return m_Camera; }
  
  virtual void MouseLeftDrag( int diffX, int diffY ) = 0;
  virtual void MouseMiddleDrag( int diffX, int diffY ) = 0;
  virtual void MouseRightDrag( int diffX, int diffY ) = 0;
    
protected:
  Ogre::Camera* m_Camera;
  Ogre::SceneManager* m_SceneManager;
};   
  
} // namespace ogre_tools

#endif /*OGRE_TOOLS_CAMERA_BASE_H_*/
