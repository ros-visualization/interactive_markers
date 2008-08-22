#include "camera_base.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

CameraBase::CameraBase( Ogre::SceneManager* scene_manager )
: scene_manager_( scene_manager )
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

void CameraBase::setPosition( const Ogre::Vector3& position )
{
  setPosition( position.x, position.y, position.z );
}

void CameraBase::setOrientation( const Ogre::Quaternion& orientation )
{
  setOrientation( orientation.x, orientation.y, orientation.z, orientation.w );
}

} // namespace ogre_tools
