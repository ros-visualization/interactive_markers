#ifndef OGRE_TOOLS_ORBIT_CAMERA_H_
#define OGRE_TOOLS_ORBIT_CAMERA_H_

#include "camera_base.h"

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
  
  virtual void Yaw( float angle );
  virtual void Pitch( float angle );
  virtual void Roll( float angle );
  virtual void SetOrientation( float x, float y, float z, float w );
  
  /// Move the camera relative to its forward axis
  virtual void Move( float x, float y, float z );
  virtual void SetPosition( float x, float y, float z );
  
  virtual void MouseLeftDrag( int diffX, int diffY );
  virtual void MouseMiddleDrag( int diffX, int diffY );
  virtual void MouseRightDrag( int diffX, int diffY );
    
protected:
  Ogre::SceneNode* m_PositionNode;
  Ogre::SceneNode* m_PivotNode;
  Ogre::SceneNode* m_YawNode;
  Ogre::SceneNode* m_PitchNode;
  Ogre::SceneNode* m_RollNode;
};   
  
} // namespace ogre_tools

#endif /*OGRE_TOOLS_ORBIT_CAMERA_H_*/
