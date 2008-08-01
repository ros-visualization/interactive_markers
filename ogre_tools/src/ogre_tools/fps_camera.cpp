#include "fps_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{
    
FPSCamera::FPSCamera( Ogre::SceneManager* sceneManager )
: CameraBase( sceneManager )
{ 
  m_PositionNode = sceneManager->getRootSceneNode()->createChildSceneNode();
  m_YawNode = m_PositionNode->createChildSceneNode();
  m_PitchNode = m_YawNode->createChildSceneNode();
  m_RollNode = m_PitchNode->createChildSceneNode();
  
  m_RollNode->attachObject( m_Camera );
}

FPSCamera::~FPSCamera()
{ 
  m_PositionNode->getParentSceneNode()->removeAndDestroyChild( m_PositionNode->getName() );
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

void FPSCamera::MouseLeftDrag( int diffX, int diffY )
{
  Yaw( -diffX*0.005 );
  Pitch( -diffY*0.005 );
}

void FPSCamera::MouseMiddleDrag( int diffX, int diffY )
{
  Move( diffX*0.1, -diffY*0.1, 0.0f );
}

void FPSCamera::MouseRightDrag( int diffX, int diffY )
{
  Move( 0.0f, 0.0f, diffY*0.1 );
}

} // namespace ogre_tools
