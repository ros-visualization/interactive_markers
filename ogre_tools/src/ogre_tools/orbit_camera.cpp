#include "orbit_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float YAW_START = Ogre::Math::PI - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI;

namespace ogre_tools
{

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

void OrbitCamera::SetOrientation( float x, float y, float z, float w )
{
  Ogre::Quaternion quat( w, x, y, z );
  m_Camera->setOrientation( quat );

  m_FocalPoint = quat * (Ogre::Vector3::NEGATIVE_UNIT_Z * m_Distance);

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

  m_Pitch = acos( z / m_Distance ) + PITCH_START;
  NormalizePitch();

  m_Yaw = acos( x /( m_Distance * sin( m_Pitch ) ) );

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
