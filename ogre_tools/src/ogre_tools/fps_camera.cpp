/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "fps_camera.h"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

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

void FPSCamera::relativeNodeChanged()
{
  if ( relative_node_ )
  {
    relative_node_->attachObject( camera_ );
  }
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

void FPSCamera::lookAt( const Ogre::Vector3& point )
{
  camera_->lookAt( point );

  CameraBase::setOrientation( camera_->getOrientation() );

  update();
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
