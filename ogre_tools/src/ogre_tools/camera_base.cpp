#include "camera_base.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{
    
CameraBase::CameraBase( Ogre::SceneManager* sceneManager )
: m_SceneManager( sceneManager )
{
  std::stringstream ss;
  static uint32_t count = 0;
  ss << "FPSCamera" << count++;
  m_Camera = m_SceneManager->createCamera( ss.str() );
}

CameraBase::~CameraBase()
{
  m_SceneManager->destroyCamera( m_Camera );
}

} // namespace ogre_tools
