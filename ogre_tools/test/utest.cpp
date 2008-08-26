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

/* Author: Josh Faust */

#include <gtest/gtest.h>
#include "../src/ogre_tools/fps_camera.h"
#include "../src/ogre_tools/orbit_camera.h"

#include <Ogre.h>

bool g_initialized = false;
Ogre::Root* g_root = NULL;
Ogre::SceneManager* g_scene_manager = NULL;
Ogre::Viewport* g_viewport = NULL;
Ogre::RenderWindow* g_render_window = NULL;

using namespace ogre_tools;

void initializeOgre()
{
  if ( g_initialized )
  {
    return;
  }

  g_root = new Ogre::Root();
  EXPECT_EQ( 0, 0 );
  g_root->loadPlugin( "RenderSystem_GL" );
  g_root->loadPlugin( "Plugin_OctreeSceneManager" );

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Taken from gazebo
  Ogre::RenderSystemList *rsList = g_root->getAvailableRenderers();

  Ogre::RenderSystem* render_system = NULL;
  Ogre::RenderSystemList::iterator renderIt = rsList->begin();
  Ogre::RenderSystemList::iterator renderEnd = rsList->end();
  for ( ; renderIt != renderEnd; ++renderIt )
  {
      render_system = *renderIt;

      if ( render_system->getName() == "OpenGL Rendering Subsystem" )
      {
          break;
      }
  }

  if ( render_system == NULL )
  {
    printf( "Could not find the opengl rendering subsystem!\n" );
    exit(1);
  }

  render_system->setConfigOption("Full Screen","No");
  render_system->setConfigOption("FSAA","2");
  render_system->setConfigOption("RTT Preferred Mode", "PBuffer");

  g_root->setRenderSystem( render_system );
  g_render_window = g_root->initialise( true );
  g_scene_manager = g_root->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );
  g_viewport = g_render_window->addViewport( NULL );

  g_initialized = true;
}

void cleanupOgre()
{
  g_scene_manager->clearScene();
  g_root->destroySceneManager( g_scene_manager );

  delete g_root;
}

TEST(Camera, orbitSetPosition)
{
  OrbitCamera* orbit_cam = new OrbitCamera( g_scene_manager );
  g_viewport->setCamera( orbit_cam->getOgreCamera() );

  Ogre::Vector3 expected_position( 10.0f, 442.0f, -194.0f );
  orbit_cam->CameraBase::setPosition( expected_position );
  Ogre::Vector3 actual_position = orbit_cam->getPosition();

  EXPECT_FLOAT_EQ( expected_position.x, actual_position.x );
  EXPECT_FLOAT_EQ( expected_position.y, actual_position.y );
  EXPECT_FLOAT_EQ( expected_position.z, actual_position.z );

  g_viewport->setCamera( NULL );

  delete orbit_cam;
}

TEST(Camera, orbitSetOrientation)
{
  OrbitCamera* orbit_cam = new OrbitCamera( g_scene_manager );
  g_viewport->setCamera( orbit_cam->getOgreCamera() );

  Ogre::Quaternion expected_orientation( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 0.0f, 1.0f, 0.0f ) );
  expected_orientation = expected_orientation * Ogre::Quaternion( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 1.0f, 0.0f, 0.0f ) );

  orbit_cam->CameraBase::setOrientation( expected_orientation );

  Ogre::Quaternion actual_orientation = orbit_cam->getOrientation();

  EXPECT_NEAR( expected_orientation.x, actual_orientation.x, 0.000001 );
  EXPECT_NEAR( expected_orientation.y, actual_orientation.y, 0.000001 );
  EXPECT_NEAR( expected_orientation.z, actual_orientation.z, 0.000001 );
  EXPECT_NEAR( expected_orientation.w, actual_orientation.w, 0.000001 );

  g_viewport->setCamera( NULL );

  delete orbit_cam;
}

TEST(Camera, fpsSetPosition)
{
  FPSCamera* fps_cam = new FPSCamera( g_scene_manager );
  g_viewport->setCamera( fps_cam->getOgreCamera() );

  Ogre::Vector3 expected_position( 10.0f, 442.0f, -194.0f );
  fps_cam->CameraBase::setPosition( expected_position );
  Ogre::Vector3 actual_position = fps_cam->getPosition();

  EXPECT_FLOAT_EQ( expected_position.x, actual_position.x );
  EXPECT_FLOAT_EQ( expected_position.y, actual_position.y );
  EXPECT_FLOAT_EQ( expected_position.z, actual_position.z );

  g_viewport->setCamera( NULL );

  delete fps_cam;
}

TEST(Camera, fpsSetOrientation)
{
  FPSCamera* fps_cam = new FPSCamera( g_scene_manager );
  g_viewport->setCamera( fps_cam->getOgreCamera() );

  Ogre::Quaternion expected_orientation( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 0.0f, 1.0f, 0.0f ) );
  expected_orientation = expected_orientation * Ogre::Quaternion( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 1.0f, 0.0f, 0.0f ) );

  fps_cam->CameraBase::setOrientation( expected_orientation );

  Ogre::Quaternion actual_orientation = fps_cam->getOrientation();

  EXPECT_NEAR( expected_orientation.x, actual_orientation.x, 0.000001 );
  EXPECT_NEAR( expected_orientation.y, actual_orientation.y, 0.000001 );
  EXPECT_NEAR( expected_orientation.z, actual_orientation.z, 0.000001 );
  EXPECT_NEAR( expected_orientation.w, actual_orientation.w, 0.000001 );

  g_viewport->setCamera( NULL );

  delete fps_cam;
}

int main( int argc, char** argv )
{
  testing::InitGoogleTest( &argc, argv );

  initializeOgre();
  g_root->renderOneFrame();

  int return_val = RUN_ALL_TESTS();

  cleanupOgre();

  return return_val;
}
