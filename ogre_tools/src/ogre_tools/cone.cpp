#include "cone.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>

namespace ogre_tools
{

Cone::Cone(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, int x_tes, int y_tes)
: Object( scene_manager )
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "ogre_tools::Cone" << count++;

  manual_object_ = scene_manager_->createManualObject( ss.str() );

  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  offset_node_->attachObject( manual_object_ );

  ss << "Material";
  material_name_ = ss.str();
  material_ = Ogre::MaterialManager::getSingleton().create( material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );

  create( x_tes, y_tes );

  // Default the offset so that the base of the cone is at the origin
  setOffset( Ogre::Vector3( 0.0f, 0.5f, 0.0f ) );
}

Cone::~Cone()
{
  scene_manager_->destroySceneNode( offset_node_->getName() );
  scene_manager_->destroySceneNode( scene_node_->getName() );

  scene_manager_->destroyManualObject( manual_object_ );
}


void Cone::create(int x_tes, int y_tes)
{
  manual_object_->clear();
  manual_object_->begin( material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST );

  double stepTheta = 2 * Ogre::Math::PI / x_tes;
  double stepH = 1.0 / y_tes;


  // the top and bottom
  for (int j= 0;j < x_tes;j++)
  {
    double theta = stepTheta*j;
    double h = 1.0 - stepH;

    //bottom
    {
      Ogre::Vector3 v1, v2, v3;
      Ogre::Vector3 n(0.0, -1.0, 0.0);
      v1 = Ogre::Vector3( 0.0, -0.5, 0.0 );
      getVertex( theta, 0.0, v2 );
      getVertex( theta + stepTheta, 0.0, v3 );

      manual_object_->position( v1 );
      manual_object_->normal( n );

      manual_object_->position( v2 );
      manual_object_->normal( n );

      manual_object_->position( v3 );
      manual_object_->normal( n );
    }

    //top
    {
      Ogre::Vector3 v1, v2, v3;
      v1 = Ogre::Vector3( 0.0, 0.5, 0.0 );
      getVertex( theta, h, v2 );
      getVertex( theta + stepTheta, h, v3 );

      Ogre::Vector3 n1, n2, n3;
      getNormal( theta, h, n1 );
      n2 = n1;
      getNormal( theta + stepTheta, h, n3 );

      manual_object_->position( v1 );
      manual_object_->normal( n1 );

      manual_object_->position( v3 );
      manual_object_->normal( n3 );

      manual_object_->position( v2 );
      manual_object_->normal( n2 );
    }
  }

  //the sides
  for (int i= 0;i < y_tes - 1;i++)
  {
    double h= stepH*i;
    for (int j= 0;j < x_tes;j++)
    {
      double theta= stepTheta*j;

      Ogre::Vector3 v1, v2, v3, v4;
      Ogre::Vector3 n1, n2, n3, n4;

      getVertex( theta, h, v1 );
      getVertex( theta, h + stepH, v2 );
      getVertex( theta + stepTheta, h + stepH, v3 );
      getVertex( theta + stepTheta, h, v4 );

      getNormal( theta, h, n1 );
      getNormal( theta, h + stepH, n2 );
      getNormal( theta + stepTheta, h + stepH, n3 );
      getNormal( theta + stepTheta, h, n4 );

      // 1st triangle
      manual_object_->position( v1 );
      manual_object_->normal( n1 );

      manual_object_->position( v2 );
      manual_object_->normal( n2 );

      manual_object_->position( v3 );
      manual_object_->normal( n3 );

      // 2nd triangle
      manual_object_->position( v1 );
      manual_object_->normal( n1 );

      manual_object_->position( v3 );
      manual_object_->normal( n3 );

      manual_object_->position( v4 );
      manual_object_->normal( n4 );
    }
  }

  manual_object_->end();
}

void Cone::getVertex(double theta, double h, Ogre::Vector3 & vertex)
{
  vertex.x = (1 - h)*0.5*cos(theta);
  vertex.y = h - 0.5;
  vertex.z = (1 - h)*0.5*sin(theta);

}

void Cone::getNormal(double theta, double h, Ogre::Vector3 & normal)
{
  normal.x = 0.5*cos(theta);
  normal.y = 0.5;
  normal.z = 0.5*sin(theta);
}

void Cone::setColor( float r, float g, float b, float a )
{
  material_->getTechnique(0)->setAmbient( r*0.5, g*0.5, b*0.5 );
  material_->getTechnique(0)->setDiffuse( r, g, b, a );

  if ( a < 0.9998 )
  {
    material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->setDepthWriteEnabled( false );
  }
  else
  {
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled( true );
  }
}

void Cone::setOffset( const Ogre::Vector3& offset )
{
  offset_node_->setPosition( offset );
}

void Cone::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void Cone::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void Cone::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

const Ogre::Vector3& Cone::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& Cone::getOrientation()
{
  return scene_node_->getOrientation();
}

void Cone::setUserData( const Ogre::Any& data )
{
  manual_object_->setUserAny( data );
}

} // namespace ogre_tools
