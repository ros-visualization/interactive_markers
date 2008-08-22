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

  yaw.FromAxisAngle( Ogre::Vector3::UNIT_Y, Ogre::Radian( yaw_ ) );
  pitch.FromAxisAngle( Ogre::Vector3::UNIT_X, Ogre::Radian( pitch_ ) );

  camera_->setOrientation( yaw * pitch );
}

void FPSCamera::NormalizePitch()
{
  if ( pitch_ < PITCH_LIMIT_LOW )
  {
    pitch_ = PITCH_LIMIT_LOW;
  }
  else if ( pitch_ > PITCH_LIMIT_HIGH )
  {
    pitch_ = PITCH_LIMIT_HIGH;
  }
}

void FPSCamera::NormalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void FPSCamera::Yaw( float angle )
{
  yaw_ += angle;

  NormalizeYaw();

  Update();
}

void FPSCamera::Pitch( float angle )
{
  pitch_ += angle;

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
  yaw_ = quat.getYaw( false ).valueRadians();
  pitch_ = quat.getPitch( false ).valueRadians();

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;
  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    if ( pitch_ > Ogre::Math::HALF_PI )
    {
      pitch_ = -Ogre::Math::HALF_PI + (pitch_ - Ogre::Math::HALF_PI);
    }
    else if ( pitch_ < -Ogre::Math::HALF_PI )
    {
      pitch_ = Ogre::Math::HALF_PI - (-pitch_ - Ogre::Math::HALF_PI);
    }

    yaw_ = -yaw_;

    if ( direction.dotProduct( Ogre::Vector3::UNIT_X ) < 0 )
    {
      yaw_ -= Ogre::Math::PI;
    }
    else
    {
      yaw_ += Ogre::Math::PI;
    }
  }

  NormalizePitch();
  NormalizeYaw();

  Update();
}

Ogre::Quaternion FPSCamera::GetOrientation()
{
  return camera_->getOrientation();
}

void FPSCamera::Move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );

  camera_->setPosition( camera_->getPosition() + GetOrientation() * translate );
}

void FPSCamera::SetPosition( float x, float y, float z )
{
  camera_->setPosition( x, y, z );
}

Ogre::Vector3 FPSCamera::GetPosition()
{
  return camera_->getPosition();
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
