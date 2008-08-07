#include "cone.h"

#include <Ogre.h>

namespace ogre_tools
{

Cone::Cone(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, int xTes, int yTes, float r, float g, float b)
: Object( sceneManager )
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "Cone" << count++;

  m_ManualObject = m_SceneManager->createManualObject( ss.str() );

  if ( !parentNode )
  {
    parentNode = m_SceneManager->getRootSceneNode();
  }

  m_SceneNode = parentNode->createChildSceneNode();
  m_OffsetNode = m_SceneNode->createChildSceneNode();
  m_OffsetNode->attachObject( m_ManualObject );

  ss << "Material";
  m_MaterialName = ss.str();
  m_Material = Ogre::MaterialManager::getSingleton().create( m_MaterialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  m_Material->setReceiveShadows(false);
  m_Material->getTechnique(0)->setLightingEnabled(true);
  m_Material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );

  Create( xTes, yTes, r, g, b );

  // Default the offset so that the base of the cone is at the origin
  SetOffset( Ogre::Vector3( 0.0f, 0.5f, 0.0f ) );
}

Cone::~Cone()
{
  m_SceneManager->destroySceneNode( m_OffsetNode->getName() );
  m_SceneManager->destroySceneNode( m_SceneNode->getName() );

  m_SceneManager->destroyManualObject( m_ManualObject );
}


void Cone::Create(int xTes, int yTes, float r, float g, float b)
{
  m_XTes = xTes;
  m_YTes = yTes;

  m_ManualObject->clear();
  m_ManualObject->begin( m_MaterialName, Ogre::RenderOperation::OT_TRIANGLE_LIST );

  double stepTheta = 2 * Ogre::Math::PI / xTes;
  double stepH = 1.0 / yTes;


  // the top and bottom
  for (int j= 0;j < xTes;j++)
  {
    double theta = stepTheta*j;
    double h = 1.0 - stepH;

    //bottom
    {
      Ogre::Vector3 v1, v2, v3;
      Ogre::Vector3 n(0.0, -1.0, 0.0);
      v1 = Ogre::Vector3( 0.0, -0.5, 0.0 );
      GetVertex( theta, 0.0, v2 );
      GetVertex( theta + stepTheta, 0.0, v3 );

      m_ManualObject->position( v1 );
      m_ManualObject->normal( n );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v2 );
      m_ManualObject->normal( n );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v3 );
      m_ManualObject->normal( n );
      m_ManualObject->colour( r, g, b );
    }

    //top
    {
      Ogre::Vector3 v1, v2, v3;
      v1 = Ogre::Vector3( 0.0, 0.5, 0.0 );
      GetVertex( theta, h, v2 );
      GetVertex( theta + stepTheta, h, v3 );

      Ogre::Vector3 n1, n2, n3;
      GetNormal( theta, h, n1 );
      n2 = n1;
      GetNormal( theta + stepTheta, h, n3 );

      m_ManualObject->position( v1 );
      m_ManualObject->normal( n1 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v3 );
      m_ManualObject->normal( n3 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v2 );
      m_ManualObject->normal( n2 );
      m_ManualObject->colour( r, g, b );
    }
  }

  //the sides
  for (int i= 0;i < yTes - 1;i++)
  {
    double h= stepH*i;
    for (int j= 0;j < xTes;j++)
    {
      double theta= stepTheta*j;

      Ogre::Vector3 v1, v2, v3, v4;
      Ogre::Vector3 n1, n2, n3, n4;

      GetVertex( theta, h, v1 );
      GetVertex( theta, h + stepH, v2 );
      GetVertex( theta + stepTheta, h + stepH, v3 );
      GetVertex( theta + stepTheta, h, v4 );

      GetNormal( theta, h, n1 );
      GetNormal( theta, h + stepH, n2 );
      GetNormal( theta + stepTheta, h + stepH, n3 );
      GetNormal( theta + stepTheta, h, n4 );

      // 1st triangle
      m_ManualObject->position( v1 );
      m_ManualObject->normal( n1 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v2 );
      m_ManualObject->normal( n2 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v3 );
      m_ManualObject->normal( n3 );
      m_ManualObject->colour( r, g, b );

      // 2nd triangle
      m_ManualObject->position( v1 );
      m_ManualObject->normal( n1 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v3 );
      m_ManualObject->normal( n3 );
      m_ManualObject->colour( r, g, b );

      m_ManualObject->position( v4 );
      m_ManualObject->normal( n4 );
      m_ManualObject->colour( r, g, b );
    }
  }

  m_ManualObject->end();
}

void Cone::GetVertex(double theta, double h, Ogre::Vector3 & vertex)
{
  vertex.x = (1 - h)*0.5*cos(theta);
  vertex.y = h - 0.5;
  vertex.z = (1 - h)*0.5*sin(theta);

}

void Cone::GetNormal(double theta, double h, Ogre::Vector3 & normal)
{
  normal.x = 0.5*cos(theta);
  normal.y = 0.5;
  normal.z = 0.5*sin(theta);
}

void Cone::SetColor( float r, float g, float b )
{
  m_Material->getTechnique(0)->setAmbient( r, g, b );
}

void Cone::SetOffset( const Ogre::Vector3& offset )
{
  m_OffsetNode->setPosition( offset );
}

void Cone::SetPosition( const Ogre::Vector3& position )
{
  m_SceneNode->setPosition( position );
}

void Cone::SetOrientation( const Ogre::Quaternion& orientation )
{
  m_SceneNode->setOrientation( orientation );
}

void Cone::SetScale( const Ogre::Vector3& scale )
{
  m_SceneNode->setScale( scale );
}

} // namespace ogre_tools
