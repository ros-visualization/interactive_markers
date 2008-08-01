#include "orbit_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

OrbitCamera::OrbitCamera( Ogre::SceneManager* sceneManager )
: CameraBase( sceneManager )
{
  m_PivotNode = sceneManager->getRootSceneNode()->createChildSceneNode();
  m_YawNode = m_PivotNode->createChildSceneNode();
  m_PitchNode = m_YawNode->createChildSceneNode();
  m_RollNode = m_PitchNode->createChildSceneNode();
  m_PositionNode = m_RollNode->createChildSceneNode();

  m_PositionNode->attachObject( m_Camera );
}

OrbitCamera::~OrbitCamera()
{
  m_PivotNode->getParentSceneNode()->removeAndDestroyChild( m_PivotNode->getName() );
}

void OrbitCamera::Yaw( float angle )
{
  m_YawNode->yaw( Ogre::Radian( angle ) );
}

void OrbitCamera::Pitch( float angle )
{
  m_PitchNode->pitch( Ogre::Radian( angle ) );
}

void OrbitCamera::Roll( float angle )
{
  m_RollNode->roll( Ogre::Radian( angle ) );
}

void OrbitCamera::SetOrientation( float x, float y, float z, float w )
{
  m_YawNode->setOrientation( Ogre::Quaternion::IDENTITY );
  m_PitchNode->setOrientation( Ogre::Quaternion::IDENTITY );
  m_RollNode->setOrientation( Ogre::Quaternion::IDENTITY );

  Ogre::Quaternion quat( w, x, y, z );
  m_YawNode->yaw( quat.getYaw() );
  m_PitchNode->pitch( quat.getPitch() );
  m_RollNode->roll( quat.getRoll() );
}

void OrbitCamera::Move( float x, float y, float z )
{
  m_PivotNode->translate( Ogre::Vector3( x, y, 0.0f ), Ogre::SceneNode::TS_LOCAL );
  m_PositionNode->translate( Ogre::Vector3( 0.0f, 0.0f, z ), Ogre::SceneNode::TS_LOCAL );
}

void OrbitCamera::SetPosition( float x, float y, float z )
{
  m_PositionNode->setPosition( x, y, z );
}

void OrbitCamera::MouseLeftDrag( int diffX, int diffY )
{
  Yaw( -diffX*0.005 );
  Pitch( -diffY*0.005 );
}

void OrbitCamera::MouseMiddleDrag( int diffX, int diffY )
{
}

void OrbitCamera::MouseRightDrag( int diffX, int diffY )
{
  Move( 0.0f, 0.0f, diffY*.1 );
}

} // namespace ogre_tools
