#include "camera_base.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

CameraBase::CameraBase( Ogre::SceneManager* sceneManager )
: scene_manager_( sceneManager )
{
  std::stringstream ss;
  static uint32_t count = 0;
  ss << "FPSCamera" << count++;
  camera_ = scene_manager_->createCamera( ss.str() );
}

CameraBase::~CameraBase()
{
  scene_manager_->destroyCamera( camera_ );
}

void CameraBase::SetPosition( const Ogre::Vector3& position )
{
  SetPosition( position.x, position.y, position.z );
}

void CameraBase::SetOrientation( const Ogre::Quaternion& orientation )
{
  SetOrientation( orientation.x, orientation.y, orientation.z, orientation.w );
}

} // namespace ogre_tools
