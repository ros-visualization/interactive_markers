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
#include "../src/ogre_tools/initialization.h"

#include <ros/common.h>

#include <Ogre.h>

bool g_initialized = false;
Ogre::Root* g_root = NULL;
Ogre::SceneManager* g_scene_manager = NULL;
Ogre::Viewport* g_viewport = NULL;
Ogre::RenderWindow* g_render_window = NULL;

using namespace ogre_tools;

class TestEnvironment : public testing::Environment
{
public:
  virtual void SetUp()
  {
    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog( "Ogre.log", false, false, true );

    g_root = new Ogre::Root();
    g_root->loadPlugin( "RenderSystem_GL" );
    g_root->loadPlugin( "Plugin_OctreeSceneManager" );

    std::string ogre_tools_path = ros::getPackagePath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/models", "FileSystem", ROS_PACKAGE_NAME );
    ogre_tools::initializeResources(ogre_tools::V_string());

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

    g_root->renderOneFrame();
  }

  virtual void TearDown()
  {
    g_scene_manager->clearScene();
    g_root->destroySceneManager( g_scene_manager );

    delete g_root;
  }
};

#define EXPECT_VECTOR_EQ( expected, actual ) \
  EXPECT_FLOAT_EQ( expected.x, actual.x ); \
  EXPECT_FLOAT_EQ( expected.y, actual.y ); \
  EXPECT_FLOAT_EQ( expected.z, actual.z );

#define EXPECT_VECTOR_NEAR( expected, actual, tolerance ) \
  EXPECT_NEAR( expected.x, actual.x, tolerance ); \
  EXPECT_NEAR( expected.y, actual.y, tolerance ); \
  EXPECT_NEAR( expected.z, actual.z, tolerance );

#define EXPECT_QUATERNION_EQ( expected, actual ) \
  EXPECT_FLOAT_EQ( expected.x, actual.x ); \
  EXPECT_FLOAT_EQ( expected.y, actual.y ); \
  EXPECT_FLOAT_EQ( expected.z, actual.z ); \
  EXPECT_FLOAT_EQ( expected.w, actual.w );

#define EXPECT_QUATERNION_NEAR( expected, actual, tolerance ) \
  EXPECT_NEAR( expected.x, actual.x, tolerance ); \
  EXPECT_NEAR( expected.y, actual.y, tolerance ); \
  EXPECT_NEAR( expected.z, actual.z, tolerance ); \
  EXPECT_NEAR( expected.w, actual.w, tolerance );

TEST(OrbitCamera, setPosition)
{
  OrbitCamera* orbit_cam = new OrbitCamera( g_scene_manager );
  g_viewport->setCamera( orbit_cam->getOgreCamera() );

  Ogre::Vector3 expected_position( 10.0f, 442.0f, -194.0f );
  orbit_cam->CameraBase::setPosition( expected_position );
  Ogre::Vector3 actual_position = orbit_cam->getPosition();

  EXPECT_VECTOR_EQ( expected_position, actual_position );

  g_viewport->setCamera( NULL );

  delete orbit_cam;
}

TEST(OrbitCamera, setOrientation)
{
  OrbitCamera* orbit_cam = new OrbitCamera( g_scene_manager );
  g_viewport->setCamera( orbit_cam->getOgreCamera() );

  Ogre::Quaternion expected_orientation( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 0.0f, 1.0f, 0.0f ) );
  expected_orientation = expected_orientation * Ogre::Quaternion( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 1.0f, 0.0f, 0.0f ) );

  orbit_cam->CameraBase::setOrientation( expected_orientation );

  Ogre::Quaternion actual_orientation = orbit_cam->getOrientation();

  EXPECT_QUATERNION_NEAR( expected_orientation, actual_orientation, 0.000001 );

  g_viewport->setCamera( NULL );

  delete orbit_cam;
}

TEST(OrbitCamera, saveAndLoad)
{
  OrbitCamera* cam = new OrbitCamera( g_scene_manager );
  g_viewport->setCamera( cam->getOgreCamera() );

  const Ogre::Vector3 pos = Ogre::Vector3(0.0f, 10.0f, 0.0f);
  const Ogre::Vector3 fp = Ogre::Vector3(0.0f, 5.0f, 0.0f);
  cam->setFocalPoint(fp);
  cam->CameraBase::setPosition(pos);
  float pitch = cam->getPitch();
  float yaw = cam->getYaw();
  float distance = cam->getDistance();

  std::string str = cam->toString();
  cam->fromString(str);

  EXPECT_NEAR(pitch, cam->getPitch(), 0.001);
  EXPECT_NEAR(yaw, cam->getYaw(), 0.001);
  EXPECT_NEAR(distance, cam->getDistance(), 0.001);
  EXPECT_VECTOR_NEAR(pos, cam->getPosition(), 0.01);
  EXPECT_VECTOR_NEAR(fp, cam->getFocalPoint(), 0.001);

  g_viewport->setCamera( NULL );

  delete cam;
}

TEST(FPSCamera, setPosition)
{
  FPSCamera* fps_cam = new FPSCamera( g_scene_manager );
  g_viewport->setCamera( fps_cam->getOgreCamera() );

  Ogre::Vector3 expected_position( 10.0f, 442.0f, -194.0f );
  fps_cam->CameraBase::setPosition( expected_position );
  Ogre::Vector3 actual_position = fps_cam->getPosition();

  EXPECT_VECTOR_EQ( expected_position, actual_position );

  g_viewport->setCamera( NULL );

  delete fps_cam;
}

TEST(FPSCamera, setOrientation)
{
  FPSCamera* fps_cam = new FPSCamera( g_scene_manager );
  g_viewport->setCamera( fps_cam->getOgreCamera() );

  Ogre::Quaternion expected_orientation( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 0.0f, 1.0f, 0.0f ) );
  expected_orientation = expected_orientation * Ogre::Quaternion( Ogre::Radian( Ogre::Math::HALF_PI / 2.0f ), Ogre::Vector3( 1.0f, 0.0f, 0.0f ) );

  fps_cam->CameraBase::setOrientation( expected_orientation );

  Ogre::Quaternion actual_orientation = fps_cam->getOrientation();

  EXPECT_QUATERNION_NEAR( expected_orientation, actual_orientation, 0.000001 );

  g_viewport->setCamera( NULL );

  delete fps_cam;
}

TEST(FPSCamera, saveAndLoad)
{
  FPSCamera* cam = new FPSCamera( g_scene_manager );
  g_viewport->setCamera( cam->getOgreCamera() );

  const Ogre::Vector3 pos = Ogre::Vector3(0.0f, 10.0f, 0.0f);
  const Ogre::Vector3 fp = Ogre::Vector3(0.0f, 5.0f, 0.0f);
  cam->lookAt(fp);
  cam->CameraBase::setPosition(pos);
  float pitch = cam->getPitch();
  float yaw = cam->getYaw();

  std::string str = cam->toString();
  cam->fromString(str);

  EXPECT_NEAR(pitch, cam->getPitch(), 0.001);
  EXPECT_NEAR(yaw, cam->getYaw(), 0.001);
  EXPECT_VECTOR_NEAR(pos, cam->getPosition(), 0.001);

  g_viewport->setCamera( NULL );

  delete cam;
}

class OrbitSetFromFPS : public testing::Test
{
public:
  virtual void SetUp()
  {
    fps_cam_ = new FPSCamera( g_scene_manager );
    orbit_cam_ = new OrbitCamera( g_scene_manager );
  }

  virtual void TearDown()
  {
    delete fps_cam_;
    delete orbit_cam_;
  }

  void TestQuadrant( float x, float y, float z )
  {
    fps_cam_->CameraBase::setPosition( Ogre::Vector3( x, y, z ) );
    fps_cam_->CameraBase::setOrientation( Ogre::Quaternion( Ogre::Radian( Ogre::Math::HALF_PI ), Ogre::Vector3::UNIT_X ) );

    Ogre::Vector3 expected_position = fps_cam_->getPosition();
    Ogre::Quaternion expected_orientation = fps_cam_->getOrientation();

    orbit_cam_->setFrom( fps_cam_ );

    Ogre::Vector3 actual_position = orbit_cam_->getPosition();
    Ogre::Quaternion actual_orientation = orbit_cam_->getOrientation();

    EXPECT_VECTOR_NEAR( expected_position, actual_position, 0.001 );
    EXPECT_QUATERNION_NEAR( expected_orientation, actual_orientation, 0.001 );
  }

  FPSCamera* fps_cam_;
  OrbitCamera* orbit_cam_;
};

TEST_F(OrbitSetFromFPS, quadrantPosXPosYPosZ)
{
  TestQuadrant( 1.0f, 1.0f, 1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantPosXNegYPosZ)
{
  TestQuadrant( 1.0f, -1.0f, 1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantNegXPosYPosZ)
{
  TestQuadrant( -1.0f, 1.0f, 1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantNegXNegYPosZ)
{
  TestQuadrant( -1.0f, -1.0f, 1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantNegXPosYNegZ)
{
  TestQuadrant( -1.0f, 1.0f, -1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantNegXNegYNegZ)
{
  TestQuadrant( -1.0f, -1.0f, -1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantPosXPosYNegZ)
{
  TestQuadrant( 1.0f, 1.0f, -1.0f );
}

TEST_F(OrbitSetFromFPS, quadrantPosXNegYNegZ)
{
  TestQuadrant( 1.0f, -1.0f, -1.0f );
}

class FPSSetFromOrbit : public testing::Test
{
public:
  virtual void SetUp()
  {
    fps_cam_ = new FPSCamera( g_scene_manager );
    orbit_cam_ = new OrbitCamera( g_scene_manager );
  }

  virtual void TearDown()
  {
    delete fps_cam_;
    delete orbit_cam_;
  }

  void TestQuadrant( float x, float y, float z )
  {
    orbit_cam_->CameraBase::setPosition( Ogre::Vector3( x, y, z ) );

    Ogre::Vector3 expected_position = orbit_cam_->getPosition();
    Ogre::Quaternion expected_orientation = orbit_cam_->getOrientation();

    fps_cam_->setFrom( orbit_cam_ );

    Ogre::Vector3 actual_position = fps_cam_->getPosition();
    Ogre::Quaternion actual_orientation = fps_cam_->getOrientation();

    EXPECT_VECTOR_NEAR( expected_position, actual_position, 0.001 );
    EXPECT_QUATERNION_NEAR( expected_orientation, actual_orientation, 0.001 );
  }

  FPSCamera* fps_cam_;
  OrbitCamera* orbit_cam_;
};

TEST_F(FPSSetFromOrbit, quadrantPosXPosYPosZ)
{
  TestQuadrant( 1.0f, 1.0f, 1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantPosXNegYPosZ)
{
  TestQuadrant( 1.0f, -1.0f, 1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantNegXPosYPosZ)
{
  TestQuadrant( -1.0f, 1.0f, 1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantNegXNegYPosZ)
{
  TestQuadrant( -1.0f, -1.0f, 1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantNegXPosYNegZ)
{
  TestQuadrant( -1.0f, 1.0f, -1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantNegXNegYNegZ)
{
  TestQuadrant( -1.0f, -1.0f, -1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantPosXPosYNegZ)
{
  TestQuadrant( 1.0f, 1.0f, -1.0f );
}

TEST_F(FPSSetFromOrbit, quadrantPosXNegYNegZ)
{
  TestQuadrant( 1.0f, -1.0f, -1.0f );
}

int main( int argc, char** argv )
{
  testing::InitGoogleTest( &argc, argv );
  testing::AddGlobalTestEnvironment( new TestEnvironment );

  return RUN_ALL_TESTS();
}
