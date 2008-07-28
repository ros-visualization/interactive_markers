#include "fps_camera.h"

#include <Ogre.h>

namespace ogre_tools
{
    
FPSCamera::FPSCamera( Ogre::SceneManager* sceneManager, Ogre::Camera* camera )
: m_Camera( camera )
{
  m_PositionNode = sceneManager->getRootSceneNode()->createChildSceneNode();
  m_YawNode = m_PositionNode->createChildSceneNode();
  m_PitchNode = m_YawNode->createChildSceneNode();
  m_RollNode = m_PitchNode->createChildSceneNode();
  
  // Set ourselves from the current camera position/orientation
  Ogre::Vector3 position = m_Camera->getPosition();
  SetPosition( position.x, position.y, position.z );
  m_Camera->setPosition( Ogre::Vector3::ZERO );
  
  Ogre::Quaternion orientation = m_Camera->getOrientation();
  SetOrientation( orientation.x, orientation.y, orientation.z, orientation.w );
  m_Camera->setOrientation( Ogre::Quaternion::IDENTITY );
  
  m_RollNode->attachObject( m_Camera );
}

void FPSCamera::Yaw( float angle )
{
  m_YawNode->yaw( Ogre::Radian( angle ) );
}

void FPSCamera::Pitch( float angle )
{
  m_PitchNode->pitch( Ogre::Radian( angle ) );
}

void FPSCamera::Roll( float angle )
{
  m_RollNode->roll( Ogre::Radian( angle ) );
}

void FPSCamera::SetOrientation( float x, float y, float z, float w )
{
  m_YawNode->setOrientation( Ogre::Quaternion::IDENTITY );
  m_PitchNode->setOrientation( Ogre::Quaternion::IDENTITY );
  m_RollNode->setOrientation( Ogre::Quaternion::IDENTITY );
  
  Ogre::Quaternion quat( w, x, y, z );
  m_YawNode->yaw( quat.getYaw() );
  m_PitchNode->pitch( quat.getPitch() );
  m_RollNode->roll( quat.getRoll() );
}

void FPSCamera::Move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  
  m_PositionNode->translate( m_YawNode->getOrientation() * m_PitchNode->getOrientation() * m_RollNode->getOrientation() * translate, 
                             Ogre::SceneNode::TS_LOCAL );
  
  
}

void FPSCamera::SetPosition( float x, float y, float z )
{
  m_PositionNode->setPosition( x, y, z );
} 

} // namespace ogre_tools
