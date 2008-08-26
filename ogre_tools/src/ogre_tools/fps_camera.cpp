#include "fps_camera.h"

#include <Ogre.h>

#include <stdint.h>
#include <sstream>

namespace ogre_tools
{

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

FPSCamera::FPSCamera( Ogre::SceneManager* scene_manager )
: CameraBase( scene_manager )
{
}

FPSCamera::~FPSCamera()
{
}

void FPSCamera::update()
{
  Ogre::Matrix3 pitch, yaw;

  yaw.FromAxisAngle( Ogre::Vector3::UNIT_Y, Ogre::Radian( yaw_ ) );
  pitch.FromAxisAngle( Ogre::Vector3::UNIT_X, Ogre::Radian( pitch_ ) );

  camera_->setOrientation( yaw * pitch );
}

void FPSCamera::normalizePitch()
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

void FPSCamera::normalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void FPSCamera::yaw( float angle )
{
  yaw_ += angle;

  normalizeYaw();

  update();
}

void FPSCamera::pitch( float angle )
{
  pitch_ += angle;

  normalizePitch();

  update();
}

void FPSCamera::roll( float angle )
{
}

void FPSCamera::setFrom( CameraBase* camera )
{
  CameraBase::setPosition( camera->getPosition() );
  CameraBase::setOrientation( camera->getOrientation() );
}

void FPSCamera::setOrientation( float x, float y, float z, float w )
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

  normalizePitch();
  normalizeYaw();

  update();
}

Ogre::Quaternion FPSCamera::getOrientation()
{
  return camera_->getOrientation();
}

void FPSCamera::move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );

  camera_->setPosition( camera_->getPosition() + getOrientation() * translate );
}

void FPSCamera::setPosition( float x, float y, float z )
{
  camera_->setPosition( x, y, z );
}

Ogre::Vector3 FPSCamera::getPosition()
{
  return camera_->getPosition();
}

void FPSCamera::mouseLeftDrag( int diff_x, int diff_y )
{
  yaw( -diff_x*0.005 );
  pitch( -diff_y*0.005 );
}

void FPSCamera::mouseMiddleDrag( int diff_x, int diff_y )
{
  move( -diff_x*0.01, diff_y*0.01, 0.0f );
}

void FPSCamera::mouseRightDrag( int diff_x, int diff_y )
{
  move( 0.0f, 0.0f, diff_y*0.1 );
}

void FPSCamera::scrollWheel( int diff )
{
  move( 0.0f, 0.0f, -diff * 0.01 );
}

} // namespace ogre_tools
