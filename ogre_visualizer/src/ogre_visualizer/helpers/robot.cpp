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

#include "robot.h"
#include "common.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_tools/object.h"
#include "ogre_tools/super_ellipsoid.h"

#include <rosTF/rosTF.h>
#include <planning_models/kinematic.h>

#include <Ogre.h>

#include <rosconsole/rosconsole.h>

namespace ogre_vis
{

Robot::Robot( Ogre::SceneManager* scene_manager, const std::string& name )
: ogre_tools::Object( scene_manager )
, visual_visible_( true )
, collision_visible_( false )
, property_manager_( NULL )
, parent_property_( NULL )
, user_data_( NULL )
, name_( name )
{
  root_visual_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_collision_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_other_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode( root_visual_node_->getName() );
  scene_manager_->destroySceneNode( root_collision_node_->getName() );
}

void Robot::setVisible( bool visible )
{
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
    root_collision_node_->setVisible( collision_visible_ );
  }
  else
  {
    root_visual_node_->setVisible( false );
    root_collision_node_->setVisible( false );
  }
}

void Robot::setVisualVisible( bool visible )
{
  visual_visible_ = visible;
  root_visual_node_->setVisible( visible );
}

void Robot::setCollisionVisible( bool visible )
{
  collision_visible_ = visible;
  root_collision_node_->setVisible( visible );
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}

bool Robot::isCollisionVisible()
{
  return collision_visible_;
}

void Robot::setUserData( const Ogre::Any& user_data )
{
  user_data_ = user_data;
  M_NameToLinkInfo::iterator it = links_.begin();
  M_NameToLinkInfo::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    LinkInfo* info = it->second;

    if ( info->visual_mesh_ )
    {
      info->visual_mesh_->setUserAny( user_data_ );
    }

    if ( info->collision_mesh_ )
    {
      info->collision_mesh_->setUserAny( user_data_ );
    }

    if ( info->collision_object_ )
    {
      info->collision_object_->setUserData( user_data_ );
    }
  }
}

void Robot::clear()
{
  M_NameToLinkInfo::iterator it = links_.begin();
  M_NameToLinkInfo::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    LinkInfo* info = it->second;

    if ( info->visual_mesh_ )
    {
      scene_manager_->destroyEntity( info->visual_mesh_ );
    }

    if ( info->trail_ )
    {
      scene_manager_->destroyRibbonTrail( info->trail_ );
    }

    delete info->collision_object_;
    delete info;
  }

  if ( property_manager_ )
  {
    property_manager_->deleteByUserData( this );
  }

  links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
}

void Robot::createCollisionForLink( LinkInfo* info, robot_desc::URDF::Link* link )
{
  if ( !link->isSet[ "collision" ] )
  {
    return;
  }

  robot_desc::URDF::Link::Collision* collision = link->collision;
  robot_desc::URDF::Link::Geometry* geometry = collision->geometry;

  info->collision_node_ = root_collision_node_->createChildSceneNode();

  switch ( geometry->type )
  {
  case robot_desc::URDF::Link::Geometry::BOX:
    {
      robot_desc::URDF::Link::Geometry::Box* box = static_cast<robot_desc::URDF::Link::Geometry::Box*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info->collision_node_ );

      Ogre::Vector3 scale( box->size[0], box->size[1], box->size[2] );
      robotToOgre( scale );

      obj->create( ogre_tools::SuperEllipsoid::Cube, 10, scale );
      info->collision_object_ = obj;
    }
    break;

  case robot_desc::URDF::Link::Geometry::SPHERE:
    {
      robot_desc::URDF::Link::Geometry::Sphere* sphere = static_cast<robot_desc::URDF::Link::Geometry::Sphere*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info->collision_node_ );

      Ogre::Vector3 scale( sphere->radius, sphere->radius, sphere->radius );
      // No need to convert robot->ogre because a sphere is uniform

      obj->create( ogre_tools::SuperEllipsoid::Sphere, 10, scale );
      info->collision_object_ = obj;
    }
    break;

  case robot_desc::URDF::Link::Geometry::CYLINDER:
    {
      robot_desc::URDF::Link::Geometry::Cylinder* cylinder = static_cast<robot_desc::URDF::Link::Geometry::Cylinder*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info->collision_node_ );
      Ogre::Vector3 scale( cylinder->radius*2, cylinder->length, cylinder->radius*2 );

      obj->create( ogre_tools::SuperEllipsoid::Cylinder, 10, scale );

      info->collision_object_ = obj;
    }
    break;

  default:
    printf( "Unsupported collision shape type: %d\n", geometry->type );
  }

  if ( info->collision_object_ )
  {
    Ogre::Vector3 position( collision->xyz[0], collision->xyz[1], collision->xyz[2] );
    Ogre::Matrix3 orientation;
    orientation.FromEulerAnglesYXZ( Ogre::Radian( collision->rpy[2] ), Ogre::Radian( collision->rpy[1] ), Ogre::Radian( collision->rpy[0] ) );

    info->collision_offset_position_ = position;
    info->collision_offset_orientation_ = orientation;

    info->collision_object_->setColor( 0.0f, 0.6f, 1.0f, 1.0f );

    if ( !user_data_.isEmpty() )
    {
      info->collision_object_->setUserData( user_data_ );
    }
  }
}

void Robot::createVisualForLink( ogre_vis::Robot::LinkInfo* info, robot_desc::URDF::Link* link )
{
  robot_desc::URDF::Link::Geometry::Mesh* mesh = static_cast<robot_desc::URDF::Link::Geometry::Mesh*>(link->visual->geometry->shape);
  if ( mesh->filename.empty() )
  {
    return;
  }

  std::string model_name = mesh->filename + "_hi.mesh";

  static int count = 0;
  std::stringstream ss;
  ss << "RobotVis" << count++ << " Link " << link->name ;

  try
  {
    info->visual_mesh_ = scene_manager_->createEntity( ss.str(), model_name );
  }
  catch( Ogre::Exception& e )
  {
    printf( "Could not load model '%s' for link '%s': %s\n", model_name.c_str(), link->name.c_str(), e.what() );
  }

  if ( info->visual_mesh_ )
  {
    if ( !user_data_.isEmpty() )
    {
      info->visual_mesh_->setUserAny( user_data_ );
    }

    info->visual_node_ = root_visual_node_->createChildSceneNode();
    info->visual_node_->attachObject( info->visual_mesh_ );

    // assign the material from the link
    uint16_t num_sub_entities = info->visual_mesh_->getNumSubEntities();
    for ( uint16_t i = 0; i < num_sub_entities; ++i )
    {
      Ogre::SubEntity* subEntity = info->visual_mesh_->getSubEntity( i );

      typedef std::vector<std::string> V_string;
      V_string gazebo_names;
      link->visual->data.getMapTagNames("gazebo", gazebo_names);

      V_string::iterator name_it = gazebo_names.begin();
      V_string::iterator name_end = gazebo_names.end();
      for ( ; name_it != name_end; ++name_it )
      {
        typedef std::map<std::string, std::string> M_string;
        M_string m = link->visual->data.getMapTagValues("gazebo", *name_it);

        M_string::iterator it = m.find( "material" );
        if ( it != m.end() )
        {
          info->material_name_ = it->second;
          subEntity->setMaterialName( info->material_name_ );
          break;
        }
      }
    }
  }
}

void Robot::setPropertyManager( PropertyManager* property_manager, CategoryProperty* parent )
{
  ROS_ASSERT( property_manager );
  ROS_ASSERT( parent );

  property_manager_ = property_manager;
  parent_property_ = parent;

  links_category_ = property_manager_->createCategory( "Links", parent_property_ );

  if ( !links_.empty() )
  {
    M_NameToLinkInfo::iterator link_it = links_.begin();
    M_NameToLinkInfo::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
      LinkInfo* info = link_it->second;

      createPropertiesForLink( info );
    }
  }
}

void Robot::createPropertiesForLink( LinkInfo* info )
{
  ROS_ASSERT( property_manager_ );

  property_manager_->deleteProperty( info->position_property_ );
  property_manager_->deleteProperty( info->orientation_property_ );

  CategoryProperty* cat = property_manager_->createCategory( info->name_, links_category_, this );

  std::stringstream ss;
  ss << name_ << " Link " << info->name_;
  info->position_property_ = property_manager_->createProperty<Vector3Property>( "Position", ss.str(), boost::bind( &Robot::getPositionForLinkInRobotFrame, this, info ),
                                                                                Vector3Property::Setter(), cat, this );
  info->position_property_->setSave( false );

  info->orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", ss.str(), boost::bind( &Robot::getOrientationForLinkInRobotFrame, this, info ),
                                                                                      QuaternionProperty::Setter(), cat, this );
  info->orientation_property_->setSave( false );

  info->trail_property_ = property_manager_->createProperty<BoolProperty>( "Show Trail", ss.str(), boost::bind( &Robot::isShowingTrail, this, info ),
                                                                          boost::bind( &Robot::setShowTrail, this, info, _1 ), cat, this );

  property_manager_->getPropertyGrid()->Collapse( links_category_->getPGProperty() );
}

void Robot::load( robot_desc::URDF* urdf, bool visual, bool collision )
{
  clear();

  typedef std::vector<robot_desc::URDF::Link*> V_Link;
  V_Link links;
  urdf->getLinks(links);

  V_Link::iterator link_it = links.begin();
  V_Link::iterator link_end = links.end();
  for ( ; link_it != link_end; ++link_it )
  {
    robot_desc::URDF::Link* link = *link_it;

    LinkInfo* info = new LinkInfo;
    info->name_ = link->name;

    bool inserted = links_.insert( std::make_pair( info->name_, info ) ).second;
    ROS_ASSERT( inserted );

    if ( visual )
    {
      createVisualForLink( info, link );
    }

    if ( collision )
    {
      createCollisionForLink( info, link );
    }

    if ( property_manager_ )
    {
      createPropertiesForLink( info );
    }
  }

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());
}

Ogre::Vector3 Robot::getPositionForLinkInRobotFrame( const LinkInfo* info )
{
  ROS_ASSERT( info );

  Ogre::Vector3 pos( info->position_ );
  ogreToRobot( pos );

  return pos;
}

Ogre::Quaternion Robot::getOrientationForLinkInRobotFrame( const LinkInfo* info )
{
  ROS_ASSERT( info );

  Ogre::Quaternion orient( info->orientation_ );
  ogreToRobot( orient );

  return orient;
}

Robot::LinkInfo* Robot::getLinkInfo( const std::string& name )
{
  M_NameToLinkInfo::iterator it = links_.find( name );
  if ( it == links_.end() )
  {
    ROS_WARN( "Link %s does not exist", name.c_str() );
    return NULL;
  }

  return it->second;
}

void Robot::setShowTrail( LinkInfo* info, bool show )
{
  ROS_ASSERT( info );

  if ( show )
  {
    if ( !info->trail_ )
    {
      if ( info->visual_node_ )
      {
        static int count = 0;
        std::stringstream ss;
        ss << "Trail for link " << info->name_ << count++;
        info->trail_ = scene_manager_->createRibbonTrail( ss.str() );
        info->trail_->setMaxChainElements( 100 );
        info->trail_->setInitialWidth( 0, 0.01f );
        info->trail_->setInitialColour( 0, 0.0f, 0.5f, 0.5f );
        info->trail_->addNode( info->visual_node_ );
        info->trail_->setTrailLength( 2.0f );
        root_other_node_->attachObject( info->trail_ );
      }
      else
      {
        ROS_WARN( "No visual node for link %s, cannot create a trail", info->name_.c_str() );
      }
    }
  }
  else
  {
    if ( info->trail_ )
    {
      scene_manager_->destroyRibbonTrail( info->trail_ );
      info->trail_ = NULL;
    }
  }

  info->trail_property_->changed();
}

bool Robot::isShowingTrail( const LinkInfo* info )
{
  ROS_ASSERT( info );

  return info->trail_ != NULL;
}

void setTransformsOnLink( Robot::LinkInfo* info, const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                          const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation, bool applyOffsetTransforms )
{
  info->position_ = visual_position;
  info->orientation_ = visual_orientation;

  if ( info->visual_node_ )
  {
    info->visual_node_->setPosition( visual_position );
    info->visual_node_->setOrientation( visual_orientation );
  }

  if ( info->collision_node_ )
  {
    Ogre::Quaternion initial_orientation;
    ogreToRobot( initial_orientation );

    if ( applyOffsetTransforms )
    {
      info->collision_object_->setPosition( info->collision_offset_position_ );
      info->collision_object_->setOrientation( initial_orientation * info->collision_offset_orientation_ );
    }
    else
    {
      info->collision_object_->setPosition( Ogre::Vector3::ZERO );
      info->collision_object_->setOrientation( initial_orientation );
    }

    info->collision_node_->setPosition( collision_position );
    info->collision_node_->setOrientation( collision_orientation );
  }

  if ( info->position_property_ )
  {
    info->position_property_->changed();
  }

  if ( info->orientation_property_ )
  {
    info->orientation_property_->changed();
  }
}

void Robot::update( rosTFClient* tf_client, const std::string& target_frame )
{
  M_NameToLinkInfo::iterator link_it = links_.begin();
  M_NameToLinkInfo::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const std::string& name = link_it->first;
    LinkInfo* info = link_it->second;

    libTF::TFPose pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, name };

    try
    {
      pose = tf_client->transformPose( target_frame, pose );
    }
    catch(libTF::Exception& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s'\n", name.c_str(), target_frame.c_str() );
    }

    //printf( "Link %s:\npose: %6f %6f %6f,\t%6f %6f %6f\n", name.c_str(), pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll );

    Ogre::Vector3 position( pose.x, pose.y, pose.z );
    robotToOgre( position );

    Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( pose.yaw, pose.pitch, pose.roll ) );

    // Collision/visual transforms are the same in this case
    setTransformsOnLink( info, position, orientation, position, orientation, true );
  }
}

void Robot::update( planning_models::KinematicModel* kinematic_model, const std::string& target_frame )
{
  M_NameToLinkInfo::iterator link_it = links_.begin();
  M_NameToLinkInfo::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const std::string& name = link_it->first;
    LinkInfo* info = link_it->second;

    planning_models::KinematicModel::Link* link = kinematic_model->getLink( name );

    if ( !link )
    {
      continue;
    }

    libTF::Position robot_visual_position = link->globalTransFwd.getPosition();
    libTF::Quaternion robot_visual_orientation = link->globalTransFwd.getQuaternion();
    Ogre::Vector3 visual_position( robot_visual_position.x, robot_visual_position.y, robot_visual_position.z );
    Ogre::Quaternion visual_orientation( robot_visual_orientation.w, robot_visual_orientation.x, robot_visual_orientation.y, robot_visual_orientation.z );
    robotToOgre( visual_position );
    robotToOgre( visual_orientation );

    libTF::Position robot_collision_position = link->globalTrans.getPosition();
    libTF::Quaternion robot_collision_orientation = link->globalTrans.getQuaternion();
    Ogre::Vector3 collision_position( robot_collision_position.x, robot_collision_position.y, robot_collision_position.z );
    Ogre::Quaternion collision_orientation( robot_collision_orientation.w, robot_collision_orientation.x, robot_collision_orientation.y, robot_collision_orientation.z );
    robotToOgre( collision_position );
    robotToOgre( collision_orientation );

    setTransformsOnLink( info, visual_position, visual_orientation, collision_position, collision_orientation, false );
  }
}

void Robot::setPosition( const Ogre::Vector3& position )
{
  root_visual_node_->setPosition( position );
  root_collision_node_->setPosition( position );
}

void Robot::setOrientation( const Ogre::Quaternion& orientation )
{
  root_visual_node_->setOrientation( orientation );
  root_collision_node_->setOrientation( orientation );
}

void Robot::setScale( const Ogre::Vector3& scale )
{
  root_visual_node_->setScale( scale );
  root_collision_node_->setScale( scale );
}

const Ogre::Vector3& Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}

void Robot::setColor( float r, float g, float b, float a )
{
  /// @todo Make this work on the meshes as well?

  M_NameToLinkInfo::iterator link_it = links_.begin();
  M_NameToLinkInfo::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const LinkInfo* info = link_it->second;
    info->collision_object_->setColor( r, g, b, a );
  }
}

} // namespace ogre_vis
