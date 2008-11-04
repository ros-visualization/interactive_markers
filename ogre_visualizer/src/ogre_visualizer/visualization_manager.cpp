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

#include "visualization_manager.h"
#include "visualization_panel.h"

#include "visualizer_base.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "common.h"
#include "factory.h"
#include "new_display_dialog.h"

#include "tools/tool.h"
#include "tools/move_tool.h"
#include "tools/pose_tool.h"

#include <ogre_tools/wx_ogre_render_window.h>
#include <ogre_tools/camera_base.h>

#include <ros/common.h>
#include <ros/node.h>
#include <tf/transform_listener.h>

#include <wx/timer.h>
#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>

#include <boost/bind.hpp>

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreParticleSystem.h>
#include <OgreParticle.h>
#include <OgreLight.h>

namespace ogre_vis
{

VisualizationManager::VisualizationManager( VisualizationPanel* panel )
: ogre_root_( Ogre::Root::getSingletonPtr() )
, current_tool_( NULL )
, vis_panel_( panel )
, needs_reset_( false )
{
  initializeCommon();
  registerFactories( this );

  ros_node_ = ros::node::instance();

  /// @todo This should go away once creation of the ros::node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::node( "VisualizationManager", ros::node::DONT_HANDLE_SIGINT );
  }
  ROS_ASSERT( ros_node_ );

  tf_ = new tf::TransformListener( *ros_node_, true, 10000000000ULL, 1000000000ULL );

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );
  ray_scene_query_ = scene_manager_->createRayQuery( Ogre::Ray() );

  target_relative_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateRelativeNode();

  selection_bounds_particle_system_ = scene_manager_->createParticleSystem( "VisualizationManagerSelectionBoundsParticleSystem" );
  selection_bounds_particle_system_->setMaterialName( "BaseWhiteNoLighting" );
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  node->attachObject( selection_bounds_particle_system_ );

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  update_timer_ = new wxTimer( this );
  update_timer_->Start( 10 );
  update_stopwatch_.Start();
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );

  property_manager_ = new PropertyManager( vis_panel_->getPropertyGrid() );

  CategoryProperty* category = property_manager_->createCategory( "Global Options", "", NULL );
  target_frame_property_ = property_manager_->createProperty<StringProperty>( "Target Frame", "", boost::bind( &VisualizationManager::getTargetFrame, this ),
                                                                              boost::bind( &VisualizationManager::setTargetFrame, this, _1 ), category );
  fixed_frame_property_ = property_manager_->createProperty<StringProperty>( "Fixed Frame", "", boost::bind( &VisualizationManager::getFixedFrame, this ),
                                                                                boost::bind( &VisualizationManager::setFixedFrame, this, _1 ), category );

  setTargetFrame( "base" );
  setFixedFrame( "map" );

  ros_node_->subscribe( "/time", time_message_, &VisualizationManager::incomingROSTime, this, 1 );
}

VisualizationManager::~VisualizationManager()
{
  ros_node_->unsubscribe( "/time", &VisualizationManager::incomingROSTime, this );

  Disconnect( wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );
  delete update_timer_;

  vis_panel_->getPropertyGrid()->Freeze();

  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = vis_it->visualizer_;
    delete visualizer;
  }
  visualizers_.clear();

  M_Factory::iterator factory_it = factories_.begin();
  M_Factory::iterator factory_end = factories_.end();
  for ( ; factory_it != factory_end; ++factory_it )
  {
    delete factory_it->second;
  }
  factories_.clear();

  V_Tool::iterator tool_it = tools_.begin();
  V_Tool::iterator tool_end = tools_.end();
  for ( ; tool_it != tool_end; ++tool_it )
  {
    delete *tool_it;
  }
  tools_.clear();

  delete tf_;

  delete property_manager_;
  vis_panel_->getPropertyGrid()->Thaw();

  scene_manager_->destroyParticleSystem( selection_bounds_particle_system_ );
  scene_manager_->destroyQuery( ray_scene_query_ );
  ogre_root_->destroySceneManager( scene_manager_ );
}

void VisualizationManager::initialize()
{
  MoveTool* move_tool = createTool< MoveTool >( "Move Camera" );
  setCurrentTool( move_tool );
  setDefaultTool( move_tool );

  PoseTool* goal_tool = createTool< PoseTool >( "Set Goal" );
  goal_tool->setIsGoal( true );

  createTool< PoseTool >( "Set Pose" );
}

VisualizationManager::VisualizerInfo* VisualizationManager::getVisualizerInfo( const VisualizerBase* visualizer )
{
  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* it_visualizer = vis_it->visualizer_;

    if ( visualizer == it_visualizer )
    {
     return &(*vis_it);
    }
  }

  return NULL;
}

void VisualizationManager::onUpdate( wxTimerEvent& event )
{
  long millis = update_stopwatch_.Time();
  float dt = millis / 1000.0f;

  update_stopwatch_.Start();

  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = vis_it->visualizer_;

    if ( visualizer->isEnabled() )
    {
      visualizer->update( dt );
    }
  }

  static float render_update_time = 0.0f;
  render_update_time += dt;
  if ( selection_bounds_particle_system_->getNumParticles() > 0 && render_update_time > 0.1 )
  {
    vis_panel_->queueRender();
    render_update_time = 0.0f;
  }

  updateRelativeNode();

  vis_panel_->getCurrentCamera()->update();

  if ( needs_reset_ )
  {
    needs_reset_ = false;
    resetVisualizers();
    tf_->clear();
  }
}

void VisualizationManager::addVisualizer( VisualizerBase* visualizer, bool allow_deletion, bool enabled )
{
  VisualizerInfo info;
  info.visualizer_ = visualizer;
  info.allow_deletion_ = allow_deletion;
  visualizers_.push_back( info );

  visualizer->setRenderCallback( boost::bind( &VisualizationPanel::queueRender, vis_panel_ ) );
  visualizer->setLockRenderCallback( boost::bind( &VisualizationPanel::lockRender, vis_panel_ ) );
  visualizer->setUnlockRenderCallback( boost::bind( &VisualizationPanel::unlockRender, vis_panel_ ) );

  visualizer->setTargetFrame( target_frame_ );
  visualizer->setFixedFrame( fixed_frame_ );

  vis_panel_->getPropertyGrid()->Freeze();

  std::string category_name = visualizer->getName() + " (" + visualizer->getType() + ")";
  CategoryProperty* category = property_manager_->createCategory( category_name, "", NULL );
  category->setUserData( visualizer );

  setVisualizerEnabled( visualizer, enabled );
  visualizer->setPropertyManager( property_manager_, category );

  vis_panel_->getPropertyGrid()->Thaw();
  vis_panel_->getPropertyGrid()->Refresh();
}

void VisualizationManager::resetVisualizers()
{
  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = vis_it->visualizer_;

    visualizer->reset();
  }
}

void VisualizationManager::addTool( Tool* tool )
{
  tools_.push_back( tool );

  vis_panel_->addTool( tool );
}

void VisualizationManager::setCurrentTool( Tool* tool )
{
  if ( current_tool_ )
  {
    current_tool_->deactivate();
  }

  current_tool_ = tool;
  current_tool_->activate();

  vis_panel_->setTool( tool );
}

void VisualizationManager::setDefaultTool( Tool* tool )
{
  default_tool_ = tool;
}

Tool* VisualizationManager::getTool( int index )
{
  ROS_ASSERT( index >= 0 );
  ROS_ASSERT( index < (int)tools_.size() );

  return tools_[ index ];
}

inline void createParticle( Ogre::ParticleSystem* particle_sys, const Ogre::Vector3& position, float size )
{
  Ogre::Particle* p = particle_sys->createParticle();
  if ( p )
  {
    p->setDimensions( size, size );
    p->colour = Ogre::ColourValue( 1.0f, 1.0f, 1.0f, 1.0f );
    p->timeToLive = p->totalTimeToLive = 2.0f;
    p->rotationSpeed = 0.1f;
    p->position = position;
  }
}

void createParticleLine( Ogre::ParticleSystem* particle_sys, const Ogre::Vector3& point1, const Ogre::Vector3& point2, float size, float step )
{
  createParticle( particle_sys, point1, size );

  float distance = point1.distance( point2 );
  int num_particles = distance / step;
  Ogre::Vector3 dir = point2 - point1;
  dir.normalise();
  for ( int i = 1; i < num_particles; ++i )
  {
    Ogre::Vector3 point = point1 + ( dir * ( step * i ) );

    createParticle( particle_sys, point, size );
  }

  createParticle( particle_sys, point2, size );
}

void VisualizationManager::pick( int mouse_x, int mouse_y )
{
  int width, height;
  vis_panel_->getRenderPanel()->GetSize( &width, &height );

  Ogre::Ray mouse_ray = vis_panel_->getCurrentCamera()->getOgreCamera()->getCameraToViewportRay( (float)mouse_x / (float)width, (float)mouse_y / (float)height );
  ray_scene_query_->setRay( mouse_ray );

  Ogre::RaySceneQueryResult& result = ray_scene_query_->execute();

  Ogre::MovableObject* picked = NULL;
  float closest_distance = 9999999.0f;

  // find the closest object that is also selectable
  Ogre::RaySceneQueryResult::iterator it = result.begin();
  Ogre::RaySceneQueryResult::iterator end = result.end();
  for ( ; it != end; ++it )
  {
    Ogre::RaySceneQueryResultEntry& entry = *it;

    const Ogre::Any& user_any = entry.movable->getUserAny();
    if ( user_any.isEmpty() )
    {
      continue;
    }

    // ugh -- can't just any_cast to VisualizerBase because it's abstract
    /// @todo This is dangerous, should find a better way
    const VisualizerBase* visualizer = reinterpret_cast<const VisualizerBase*>( Ogre::any_cast<void*>( user_any ) );

    if ( visualizer && visualizer->isObjectPickable( entry.movable ) )
    {
      if ( entry.distance < closest_distance )
      {
        closest_distance = entry.distance;
        picked = entry.movable;
      }
    }
  }

  if ( picked )
  {
    Ogre::SceneNode* scene_node = picked->getParentSceneNode();
    if ( scene_node )
    {
      selection_bounds_particle_system_->clear();

      const Ogre::AxisAlignedBox& aabb = scene_node->_getWorldAABB();

      const Ogre::Vector3* corners = aabb.getAllCorners();
      const float size = 0.01f;
      const float step = 0.02f;
      createParticleLine( selection_bounds_particle_system_, corners[0], corners[1], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[1], corners[2], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[2], corners[3], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[3], corners[0], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[4], corners[5], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[5], corners[6], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[6], corners[7], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[7], corners[4], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[1], corners[5], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[2], corners[4], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[3], corners[7], size, step );
      createParticleLine( selection_bounds_particle_system_, corners[0], corners[6], size, step );

      vis_panel_->getCurrentCamera()->lookAt( aabb.getCenter() );

      vis_panel_->queueRender();
    }
  }
}

VisualizerBase* VisualizationManager::getVisualizer( const std::string& name )
{
  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = vis_it->visualizer_;

    if ( visualizer->getName() == name )
    {
      return visualizer;
    }
  }

  return NULL;
}

void VisualizationManager::setVisualizerEnabled( VisualizerBase* visualizer, bool enabled )
{
  if ( enabled )
  {
    visualizer->enable();
  }
  else
  {
    visualizer->disable();
  }

  visualizer_state_( visualizer );
}

#define PROPERTY_GRID_CONFIG wxT("Property Grid State")

void VisualizationManager::loadConfig( wxConfigBase* config )
{
  int i = 0;
  while (1)
  {
    wxString type, name;
    type.Printf( wxT("Display%d/Type"), i );
    name.Printf( wxT("Display%d/Name"), i );

    wxString vis_type, vis_name;
    if ( !config->Read( type, &vis_type ) )
    {
      break;
    }

    if ( !config->Read( name, &vis_name ) )
    {
      break;
    }

    createVisualizer( (const char*)vis_type.mb_str(), (const char*)vis_name.mb_str(), false, true );

    ++i;
  }

  property_manager_->load( config );

  wxString grid_state;
  if ( config->Read( PROPERTY_GRID_CONFIG, &grid_state ) )
  {
    vis_panel_->getPropertyGrid()->RestoreEditableState( grid_state );
  }
}

void VisualizationManager::saveConfig( wxConfigBase* config )
{
  int i = 0;
  V_VisualizerInfo::iterator vis_it = visualizers_.begin();
  V_VisualizerInfo::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it, ++i )
  {
    VisualizerBase* visualizer = vis_it->visualizer_;

    wxString type, name;
    type.Printf( wxT("Display%d/Type"), i );
    name.Printf( wxT("Display%d/Name"), i );
    config->Write( type, wxString::FromAscii( visualizer->getType() ) );
    config->Write( name, wxString::FromAscii( visualizer->getName().c_str() ) );
  }

  property_manager_->save( config );

  config->Write( PROPERTY_GRID_CONFIG, vis_panel_->getPropertyGrid()->SaveEditableState() );
}

bool VisualizationManager::registerFactory( const std::string& type, VisualizerFactory* factory )
{
  M_Factory::iterator it = factories_.find( type );
  if ( it != factories_.end() )
  {
    return false;
  }

  factories_.insert( std::make_pair( type, factory ) );

  return true;
}

VisualizerBase* VisualizationManager::createVisualizer( const std::string& type, const std::string& name, bool enabled, bool allow_deletion )
{
  M_Factory::iterator it = factories_.find( type );
  if ( it == factories_.end() )
  {
    return NULL;
  }

  VisualizerBase* current_vis = getVisualizer( name );
  if ( current_vis )
  {
    return NULL;
  }

  VisualizerFactory* factory = it->second;
  VisualizerBase* visualizer = factory->create( name, this );

  addVisualizer( visualizer, allow_deletion, enabled );

  return visualizer;
}

void VisualizationManager::removeVisualizer( VisualizerBase* visualizer )
{
  V_VisualizerInfo::iterator it = visualizers_.begin();
  V_VisualizerInfo::iterator end = visualizers_.end();
  for ( ; it != end; ++it )
  {
    if ( it->visualizer_ == visualizer )
    {
      break;
    }
  }
  ROS_ASSERT( it != visualizers_.end() );

  if ( !it->allow_deletion_ )
  {
    wxMessageBox( wxT("The selected display is part of the default set of displays.  You cannot remove it."), wxT("Cannot remove.") );
    return;
  }

  visualizers_.erase( it );

  delete visualizer;

  vis_panel_->queueRender();
}

void VisualizationManager::removeVisualizer( const std::string& name )
{
  VisualizerBase* visualizer = getVisualizer( name );

  if ( !visualizer )
  {
    return;
  }

  removeVisualizer( visualizer );
}

void VisualizationManager::setTargetFrame( const std::string& frame )
{
  target_frame_ = frame;

  V_VisualizerInfo::iterator it = visualizers_.begin();
  V_VisualizerInfo::iterator end = visualizers_.end();
  for ( ; it != end; ++it )
  {
    VisualizerBase* visualizer = it->visualizer_;

    visualizer->setTargetFrame(frame);
  }

  target_frame_property_->changed();
}

void VisualizationManager::setFixedFrame( const std::string& frame )
{
  fixed_frame_ = frame;

  V_VisualizerInfo::iterator it = visualizers_.begin();
  V_VisualizerInfo::iterator end = visualizers_.end();
  for ( ; it != end; ++it )
  {
    VisualizerBase* visualizer = it->visualizer_;

    visualizer->setFixedFrame(frame);
  }

  fixed_frame_property_->changed();
}

bool VisualizationManager::isDeletionAllowed( VisualizerBase* visualizer )
{
  VisualizerInfo* info = getVisualizerInfo( visualizer );

  if ( info )
  {
    return info->allow_deletion_;
  }

  return false;
}

bool VisualizationManager::isValidVisualizer( VisualizerBase* visualizer )
{
  VisualizerInfo* info = getVisualizerInfo( visualizer );
  return info != NULL;
}

void VisualizationManager::getRegisteredTypes( std::vector<std::string>& types )
{
  M_Factory::iterator it = factories_.begin();
  M_Factory::iterator end = factories_.end();
  for ( ; it != end; ++it )
  {
    types.push_back( it->first );
  }
}

void VisualizationManager::updateRelativeNode()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                              ros::Time(0ULL), target_frame_ );

  try
  {
    tf_->transformPose( fixed_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming from frame '%s' to frame '%s'\n", target_frame_.c_str(), fixed_frame_.c_str() );
  }

  Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
  ogreToRobot( orientation );
  orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
  robotToOgre( orientation );

  target_relative_node_->setPosition( position );
  target_relative_node_->setOrientation( orientation );
}

void VisualizationManager::incomingROSTime()
{
  static ros::Time last_time = ros::Time(0ULL);

  if ( time_message_.rostime < last_time )
  {
    needs_reset_ = true;
  }

  last_time = time_message_.rostime;
}

} // namespace ogre_vis
