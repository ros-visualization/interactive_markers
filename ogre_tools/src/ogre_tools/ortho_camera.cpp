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


#include "ortho_camera.h"
#include "wx_ogre_render_window.h"

#include "rosconsole/rosassert.h"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace ogre_tools
{

OrthoCamera::OrthoCamera( wxOgreRenderWindow* render_window, Ogre::SceneManager* scene_manager )
: CameraBase( scene_manager )
, scale_( 10.0f )
, render_window_( render_window )
{
  camera_->setProjectionType( Ogre::PT_ORTHOGRAPHIC );
  camera_->setNearClipDistance(0.001f);
  camera_->setFarClipDistance(50.0f);
  camera_->setFixedYawAxis(false);

  update();
}

OrthoCamera::~OrthoCamera()
{
}

void OrthoCamera::update()
{
  render_window_->setOrthoScale( scale_ );
}

void OrthoCamera::yaw( float angle )
{
  camera_->yaw( Ogre::Radian( angle ) );
}

void OrthoCamera::pitch( float angle )
{
  camera_->pitch( Ogre::Radian( angle ) );
}

void OrthoCamera::roll( float angle )
{
  camera_->roll( Ogre::Radian( angle ) );
}

Ogre::Vector3 OrthoCamera::getPosition()
{
  return camera_->getPosition();
}

Ogre::Quaternion OrthoCamera::getOrientation()
{
  return camera_->getOrientation();
}

void OrthoCamera::setFrom( CameraBase* camera )
{
  ROS_BREAK();
}

void OrthoCamera::setOrientation( float x, float y, float z, float w )
{
  camera_->setOrientation(Ogre::Quaternion(w, x, y, z));
}

void OrthoCamera::move( float x, float y, float z )
{
  camera_->moveRelative( Ogre::Vector3( x, y, z ) );
}

void OrthoCamera::setPosition( float x, float y, float z )
{
  camera_->setPosition( x, y, z );
}

void OrthoCamera::lookAt( const Ogre::Vector3& point )
{
  camera_->lookAt( point );
}

void OrthoCamera::mouseLeftDrag( int diff_x, int diff_y )
{
  roll( -diff_x * 0.005 );
}

void OrthoCamera::mouseMiddleDrag( int diff_x, int diff_y )
{
  move( -diff_x / scale_, diff_y / scale_, 0.0f );
}

void OrthoCamera::mouseRightDrag( int diff_x, int diff_y )
{
  scale_ *= 1.0 - diff_y * 0.01;

  update();
}

void OrthoCamera::scrollWheel( int diff )
{
  scale_ *= 1.0 - (-diff) * 0.001;

  update();
}

} // namespace ogre_tools
