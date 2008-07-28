#ifndef OGRE_TOOLS_FPS_CAMERA_H_
#define OGRE_TOOLS_FPS_CAMERA_H_

namespace Ogre
{
  class Camera;
  class SceneNode;
  class SceneManager;
}

namespace ogre_tools
{
    
class FPSCamera
{
public:
  FPSCamera( Ogre::SceneManager* sceneManager, Ogre::Camera* camera );
  
  void Yaw( float angle );
  void Pitch( float angle );
  void Roll( float angle );
  void SetOrientation( float x, float y, float z, float w );
  
  /// Move the camera relative to its forward axis
  void Move( float x, float y, float z );
  void SetPosition( float x, float y, float z );
    
private:
  Ogre::Camera* m_Camera;
  
  Ogre::SceneNode* m_PositionNode;
  Ogre::SceneNode* m_YawNode;
  Ogre::SceneNode* m_PitchNode;
  Ogre::SceneNode* m_RollNode;
};   
  
} // namespace ogre_tools

#endif /*OGRE_TOOLS_FPS_CAMERA_H_*/
