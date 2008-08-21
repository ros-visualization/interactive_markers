#include "orbit_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float YAW_START = Ogre::Math::PI;// - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI;

OrbitCamera::OrbitCamera( Ogre::SceneManager* sceneManager )
: CameraBase( sceneManager )
, m_FocalPoint( 0.0f, 0.0f, 0.0f )
, m_Yaw( YAW_START )
, m_Pitch( PITCH_START )
, m_Distance( 10.0f )
{
  Update();
}

OrbitCamera::~OrbitCamera()
{
}

void OrbitCamera::NormalizePitch()
{
  if ( m_Pitch < PITCH_LIMIT_LOW )
  {
    m_Pitch = PITCH_LIMIT_LOW;
  }
  else if ( m_Pitch > PITCH_LIMIT_HIGH )
  {
    m_Pitch = PITCH_LIMIT_HIGH;
  }
}

void OrbitCamera::NormalizeYaw()
{
  m_Yaw = fmod( m_Yaw, Ogre::Math::TWO_PI );

  if ( m_Yaw < 0.0f )
  {
    m_Yaw = Ogre::Math::TWO_PI + m_Yaw;
  }
}

void OrbitCamera::Update()
{
  float x = m_Distance * cos( m_Yaw ) * sin( m_Pitch ) + m_FocalPoint.x;
  float y = m_Distance * cos( m_Pitch ) + m_FocalPoint.y;
  float z = m_Distance * sin( m_Yaw ) * sin( m_Pitch ) + m_FocalPoint.z;

  m_Camera->setPosition( x, y, z );
  m_Camera->lookAt( m_FocalPoint );
}

void OrbitCamera::Yaw( float angle )
{
  m_Yaw += angle;

  NormalizeYaw();

  Update();
}

void OrbitCamera::Pitch( float angle )
{
  m_Pitch += angle;

  NormalizePitch();

  Update();
}

void OrbitCamera::Roll( float angle )
{
}

Ogre::Vector3 OrbitCamera::GetPosition()
{
  return m_Camera->getPosition();
}

Ogre::Quaternion OrbitCamera::GetOrientation()
{
  return m_Camera->getOrientation();
}

void OrbitCamera::CalculatePitchYawFromPosition( const Ogre::Vector3& position )
{
  float x = position.x - m_FocalPoint.x;
  float y = position.y - m_FocalPoint.y;
  m_Pitch = acos( y / m_Distance );

  NormalizePitch();

  float val = x / ( m_Distance * sin( m_Pitch ) );

  m_Yaw = acos( val );

  Ogre::Vector3 direction = m_FocalPoint - position;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    m_Yaw = Ogre::Math::TWO_PI - m_Yaw;
  }
}

void OrbitCamera::SetFrom( CameraBase* camera )
{
  Ogre::Vector3 position = camera->GetPosition();
  Ogre::Quaternion orientation = camera->GetOrientation();

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * m_Distance);
  m_FocalPoint = position + direction;

  CalculatePitchYawFromPosition( position );

  Update();
}

void OrbitCamera::SetOrientation( float x, float y, float z, float w )
{
  Ogre::Quaternion quat( w, x, y, z );
  m_FocalPoint = m_Camera->getPosition() + quat * (Ogre::Vector3::NEGATIVE_UNIT_Z * m_Distance);

  m_Pitch = quat.getPitch( false ).valueRadians();
  m_Yaw = quat.getYaw( false ).valueRadians();

  NormalizePitch();
  NormalizeYaw();

  Update();
}

void OrbitCamera::Zoom( float amount )
{
  m_Distance -= amount;

  if ( m_Distance <= 0.1 )
  {
    m_Distance = 0.1;
  }

  Update();
}

void OrbitCamera::SetFocalPoint( const Ogre::Vector3& focalPoint )
{
  m_FocalPoint = focalPoint;

  Update();
}

void OrbitCamera::Move( float x, float y, float z )
{
  m_FocalPoint += m_Camera->getOrientation() * Ogre::Vector3( x, y, z );

  Update();
}

void OrbitCamera::SetPosition( float x, float y, float z )
{
  Ogre::Vector3 pos( x, y, z );
  m_Distance = (pos - m_FocalPoint).length();

  CalculatePitchYawFromPosition( Ogre::Vector3( x, y, z ) );

  Update();
}

void OrbitCamera::MouseLeftDrag( int diffX, int diffY )
{
  Yaw( diffX*0.005 );
  Pitch( -diffY*0.005 );
}

void OrbitCamera::MouseMiddleDrag( int diffX, int diffY )
{
  Move( diffX*0.1, -diffY*0.1, 0.0f );
}

void OrbitCamera::MouseRightDrag( int diffX, int diffY )
{
  Zoom( -diffY * 0.1 );
}

void OrbitCamera::ScrollWheel( int diff )
{
  Zoom( diff * 0.01 );
}

} // namespace ogre_tools
