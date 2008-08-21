#include "fps_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

FPSCamera::FPSCamera( Ogre::SceneManager* sceneManager )
: CameraBase( sceneManager )
{
}

FPSCamera::~FPSCamera()
{
}

void FPSCamera::Update()
{
  Ogre::Matrix3 pitch, yaw;

  yaw.FromAxisAngle( Ogre::Vector3::UNIT_Y, Ogre::Radian( m_Yaw ) );
  pitch.FromAxisAngle( Ogre::Vector3::UNIT_X, Ogre::Radian( m_Pitch ) );

  m_Camera->setOrientation( yaw * pitch );
}

void FPSCamera::NormalizePitch()
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

void FPSCamera::NormalizeYaw()
{
  m_Yaw = fmod( m_Yaw, Ogre::Math::TWO_PI );

  if ( m_Yaw < 0.0f )
  {
    m_Yaw = Ogre::Math::TWO_PI + m_Yaw;
  }
}

void FPSCamera::Yaw( float angle )
{
  m_Yaw += angle;

  NormalizeYaw();

  Update();
}

void FPSCamera::Pitch( float angle )
{
  m_Pitch += angle;

  NormalizePitch();

  Update();
}

void FPSCamera::Roll( float angle )
{
}

void FPSCamera::SetFrom( CameraBase* camera )
{
  CameraBase::SetPosition( camera->GetPosition() );
  CameraBase::SetOrientation( camera->GetOrientation() );
}

void FPSCamera::SetOrientation( float x, float y, float z, float w )
{
  Ogre::Quaternion quat( w, x, y, z );
  m_Yaw = quat.getYaw( false ).valueRadians();
  m_Pitch = quat.getPitch( false ).valueRadians();

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;
  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    if ( m_Pitch > Ogre::Math::HALF_PI )
    {
      m_Pitch = -Ogre::Math::HALF_PI + (m_Pitch - Ogre::Math::HALF_PI);
    }
    else if ( m_Pitch < -Ogre::Math::HALF_PI )
    {
      m_Pitch = Ogre::Math::HALF_PI - (-m_Pitch - Ogre::Math::HALF_PI);
    }

    m_Yaw = -m_Yaw;

    if ( direction.dotProduct( Ogre::Vector3::UNIT_X ) < 0 )
    {
      m_Yaw -= Ogre::Math::PI;
    }
    else
    {
      m_Yaw += Ogre::Math::PI;
    }
  }

  NormalizePitch();
  NormalizeYaw();

  Update();
}

Ogre::Quaternion FPSCamera::GetOrientation()
{
  return m_Camera->getOrientation();
}

void FPSCamera::Move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );

  m_Camera->setPosition( m_Camera->getPosition() + GetOrientation() * translate );
}

void FPSCamera::SetPosition( float x, float y, float z )
{
  m_Camera->setPosition( x, y, z );
}

Ogre::Vector3 FPSCamera::GetPosition()
{
  return m_Camera->getPosition();
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

void FPSCamera::ScrollWheel( int diff )
{

}

} // namespace ogre_tools
