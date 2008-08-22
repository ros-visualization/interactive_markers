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
  update();
}

OrbitCamera::~OrbitCamera()
{
}

void OrbitCamera::normalizePitch()
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

void OrbitCamera::normalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void OrbitCamera::update()
{
  float x = distance_ * cos( yaw_ ) * sin( pitch_ ) + focal_point_.x;
  float y = distance_ * cos( pitch_ ) + focal_point_.y;
  float z = distance_ * sin( yaw_ ) * sin( pitch_ ) + focal_point_.z;

  camera_->setPosition( x, y, z );
  camera_->lookAt( focal_point_ );
}

void OrbitCamera::yaw( float angle )
{
  yaw_ += angle;

  normalizeYaw();

  update();
}

void OrbitCamera::pitch( float angle )
{
  pitch_ += angle;

  normalizePitch();

  update();
}

void OrbitCamera::roll( float angle )
{
}

Ogre::Vector3 OrbitCamera::getPosition()
{
  return camera_->getPosition();
}

Ogre::Quaternion OrbitCamera::getOrientation()
{
  return camera_->getOrientation();
}

void OrbitCamera::calculatePitchYawFromPosition( const Ogre::Vector3& position )
{
  float x = position.x - focal_point_.x;
  float y = position.y - focal_point_.y;
  pitch_ = acos( y / distance_ );

  normalizePitch();

  float val = x / ( distance_ * sin( pitch_ ) );

  yaw_ = acos( val );

  Ogre::Vector3 direction = focal_point_ - position;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    yaw_ = Ogre::Math::TWO_PI - yaw_;
  }
}

void OrbitCamera::setFrom( CameraBase* camera )
{
  Ogre::Vector3 position = camera->getPosition();
  Ogre::Quaternion orientation = camera->getOrientation();

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);
  focal_point_ = position + direction;

  calculatePitchYawFromPosition( position );

  update();
}

void OrbitCamera::setOrientation( float x, float y, float z, float w )
{
  Ogre::Quaternion quat( w, x, y, z );
  focal_point_ = camera_->getPosition() + quat * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);

  pitch_ = quat.getPitch( false ).valueRadians();
  yaw_ = quat.getYaw( false ).valueRadians();

  normalizePitch();
  normalizeYaw();

  update();
}

void OrbitCamera::zoom( float amount )
{
  distance_ -= amount;

  if ( distance_ <= 0.1 )
  {
    distance_ = 0.1;
  }

  update();
}

void OrbitCamera::setFocalPoint( const Ogre::Vector3& focalPoint )
{
  focal_point_ = focalPoint;

  update();
}

void OrbitCamera::move( float x, float y, float z )
{
  focal_point_ += camera_->getOrientation() * Ogre::Vector3( x, y, z );

  update();
}

void OrbitCamera::setPosition( float x, float y, float z )
{
  Ogre::Vector3 pos( x, y, z );
  distance_ = (pos - focal_point_).length();

  calculatePitchYawFromPosition( Ogre::Vector3( x, y, z ) );

  update();
}

void OrbitCamera::mouseLeftDrag( int diffX, int diffY )
{
  yaw( diffX*0.005 );
  pitch( -diffY*0.005 );
}

void OrbitCamera::mouseMiddleDrag( int diffX, int diffY )
{
  move( diffX*0.1, -diffY*0.1, 0.0f );
}

void OrbitCamera::mouseRightDrag( int diffX, int diffY )
{
  zoom( -diffY * 0.1 );
}

void OrbitCamera::scrollWheel( int diff )
{
  zoom( diff * 0.01 );
}

} // namespace ogre_tools
