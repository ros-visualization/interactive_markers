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

#include "super_ellipsoid.h"

#include <cmath>

#include <Ogre.h>

#define SIGN(r) Ogre::Math::Sign(r)

namespace ogre_tools
{

SuperEllipsoid::SuperEllipsoid( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
: Object( scene_manager )
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "SuperEllipsoid" << count++;

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
}

SuperEllipsoid::~SuperEllipsoid()
{
  scene_manager_->destroySceneNode( scene_node_->getName() );
  scene_manager_->destroySceneNode( offset_node_->getName() );

  scene_manager_->destroyManualObject( manual_object_ );

  material_.setNull();
}

void SuperEllipsoid::create(int samples, float n1, float n2, const Ogre::Vector3& scale)
{
  float phi = 0.0, beta = 0.0;
  Ogre::Vector3 p1, p2, p3;

  float dB = Ogre::Math::TWO_PI/samples;
  float dP = Ogre::Math::TWO_PI/samples;

  phi = -Ogre::Math::HALF_PI;

  manual_object_->clear();
  manual_object_->begin( material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST );

  float scaleX = scale.x / 2.0f;
  float scaleY = scale.y / 2.0f;
  float scaleZ = scale.z / 2.0f;

  for(int j=0; j<=samples/2; j++)
  {
    beta = -Ogre::Math::PI;

    for(int i=0; i<=samples; i++)
    {
      //Triangle #1
      manual_object_->position(Sample(phi+dP, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi+dP, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->position(Sample(phi, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->position(Sample(phi+dP, beta+dB, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi+dP, beta+dB, n1, n2, scaleX, scaleY, scaleZ));

      //Triangle #2
      manual_object_->position(Sample(phi+dP, beta+dB, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi+dP, beta+dB, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->position(Sample(phi, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi, beta, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->position(Sample(phi, beta+dB, n1, n2, scaleX, scaleY, scaleZ));
      manual_object_->normal(CalculateNormal(phi, beta+dB, n1, n2, scaleX, scaleY, scaleZ));

      beta += dB;
    }

    phi += dP;
  }

  manual_object_->end();
}

void SuperEllipsoid::create(Shape shape, int samples, const Ogre::Vector3& scale)
{
  float n1, n2;

  switch(shape)
  {
  case Cube:
    n1 = n2 = 0.0;
    break;
  case RoundedCube:
    n1 = n2 = 0.2;
    break;
  case Cylinder:
    n1 = 0.0;
    n2 = 1.0;
    break;
  case Sphere:
    n1 = n2 = 1.0;
    break;
  }

  create(samples, n1, n2, scale);
}

Ogre::Vector3 SuperEllipsoid::Sample(float phi, float beta, float n1, float n2,
                                     float scaleX, float scaleY, float scaleZ)
{
  Ogre::Vector3 vertex;

  float cosPhi = cos(phi);
  float cosBeta = cos(beta);
  float sinPhi = sin(phi);
  float sinBeta = sin(beta);


  vertex.x = scaleX * SIGN(cosPhi) * pow(fabs(cosPhi), n1) * SIGN(sinBeta) * pow(fabs(sinBeta), n2);
  vertex.y = scaleY * SIGN(sinPhi) * pow(fabs(sinPhi), n1);
  vertex.z = scaleZ * SIGN(cosPhi) * pow(fabs(cosPhi), n1) * SIGN(cosBeta) * pow(fabs(cosBeta), n2);

  return vertex;
}

Ogre::Vector3 SuperEllipsoid::CalculateNormal(float phi, float beta, float n1, float n2,
                                              float scaleX, float scaleY, float scaleZ)
{
  Ogre::Vector3 normal;

  float cosPhi = cos(phi);
  float cosBeta = cos(beta);
  float sinPhi = sin(phi);
  float sinBeta = sin(beta);

  normal.x = SIGN(cosPhi) * pow(fabs(cosPhi), 2-n1) * SIGN(sinBeta) * pow(fabs(sinBeta), 2-n2) / scaleX;
  normal.y = SIGN(sinPhi) * pow(fabs(sinPhi), 2-n1) / scaleY;
  normal.z = SIGN(cosPhi) * pow(fabs(cosPhi), 2-n1) * SIGN(cosBeta) * pow(fabs(cosBeta), 2-n2) / scaleZ;

  normal.normalise();

  return normal;
}

void SuperEllipsoid::setColor( float r, float g, float b )
{
  material_->getTechnique(0)->setAmbient( r*0.5, g*0.5, b*0.5 );
  material_->getTechnique(0)->setDiffuse( r, g, b, 1.0f );
}

void SuperEllipsoid::setOffset( const Ogre::Vector3& offset )
{
  offset_node_->setPosition( offset );
}

void SuperEllipsoid::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void SuperEllipsoid::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void SuperEllipsoid::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( scale );
}

} // namespace ogre_tools
