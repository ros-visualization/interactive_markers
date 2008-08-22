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
, focal_point_( 0.0f, 0.0f, 0.0f )
, yaw_( YAW_START )
, pitch_( PITCH_START )
, distance_( 10.0f )
{
  Update();
}

OrbitCamera::~OrbitCamera()
{
}

void OrbitCamera::NormalizePitch()
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

void OrbitCamera::NormalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void OrbitCamera::Update()
{
  float x = distance_ * cos( yaw_ ) * sin( pitch_ ) + focal_point_.x;
  float y = distance_ * cos( pitch_ ) + focal_point_.y;
  float z = distance_ * sin( yaw_ ) * sin( pitch_ ) + focal_point_.z;

  camera_->setPosition( x, y, z );
  camera_->lookAt( focal_point_ );
}

void OrbitCamera::Yaw( float angle )
{
  yaw_ += angle;

  NormalizeYaw();

  Update();
}

void OrbitCamera::Pitch( float angle )
{
  pitch_ += angle;

  NormalizePitch();

  Update();
}

void OrbitCamera::Roll( float angle )
{
}

Ogre::Vector3 OrbitCamera::GetPosition()
{
  return camera_->getPosition();
}

Ogre::Quaternion OrbitCamera::GetOrientation()
{
  return camera_->getOrientation();
}

void OrbitCamera::CalculatePitchYawFromPosition( const Ogre::Vector3& position )
{
  float x = position.x - focal_point_.x;
  float y = position.y - focal_point_.y;
  pitch_ = acos( y / distance_ );

  NormalizePitch();

  float val = x / ( distance_ * sin( pitch_ ) );

  yaw_ = acos( val );

  Ogre::Vector3 direction = focal_point_ - position;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    yaw_ = Ogre::Math::TWO_PI - yaw_;
  }
}

void OrbitCamera::SetFrom( CameraBase* camera )
{
  Ogre::Vector3 position = camera->GetPosition();
  Ogre::Quaternion orientation = camera->GetOrientation();

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);
  focal_point_ = position + direction;

  CalculatePitchYawFromPosition( position );

  Update();
}

void OrbitCamera::SetOrientation( float x, float y, float z, float w )
{
  Ogre::Quaternion quat( w, x, y, z );
  focal_point_ = camera_->getPosition() + quat * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);

  pitch_ = quat.getPitch( false ).valueRadians();
  yaw_ = quat.getYaw( false ).valueRadians();

  NormalizePitch();
  NormalizeYaw();

  Update();
}

void OrbitCamera::Zoom( float amount )
{
  distance_ -= amount;

  if ( distance_ <= 0.1 )
  {
    distance_ = 0.1;
  }

  Update();
}

void OrbitCamera::SetFocalPoint( const Ogre::Vector3& focalPoint )
{
  focal_point_ = focalPoint;

  Update();
}

void OrbitCamera::Move( float x, float y, float z )
{
  focal_point_ += camera_->getOrientation() * Ogre::Vector3( x, y, z );

  Update();
}

void OrbitCamera::SetPosition( float x, float y, float z )
{
  Ogre::Vector3 pos( x, y, z );
  distance_ = (pos - focal_point_).length();

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
