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

/*
 * octree_visualizer.cpp
 *
 *  Created on: Aug 20, 2008
 *      Author: Matthew Piccoli and Matei Ciocarlie
 */

#include "octree_visualizer.h"
#include "octree.h"
#include "dataTypes.h"
#include "ros/node.h"
#include <rosTF/rosTF.h>

#include "../common.h"

namespace ogre_vis
{

	OctreeVisualizer::OctreeVisualizer(Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
	: VisualizerBase( sceneManager, node, tfClient, name, enabled )
	, m_R( 1.0 )
	, m_G( 0.0 )
	, m_B( 0.0 )
	, m_NewMessage( false )
	{
		static uint32_t count = 0;
		std::stringstream ss;
		ss << "Cone" << count++;

		m_ManualObject = m_SceneManager->createManualObject( ss.str() );

		m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();
		m_SceneNode->attachObject( m_ManualObject );

		if ( IsEnabled() )
		{
			OnEnable();
		}

	}

	OctreeVisualizer::~OctreeVisualizer() {
		// TODO Auto-generated destructor stub
	}

	void OctreeVisualizer::OnEnable()
	{
		m_ManualObject->setVisible( true );
		Subscribe();
	}

	void OctreeVisualizer::OnDisable()
	{
		m_ManualObject->setVisible( false );
		Unsubscribe();
	}

	void OctreeVisualizer::Subscribe()
	{
		if ( !IsEnabled() )
		  {
		    return;
		  }

		  if ( !m_OctreeTopic.empty() )
		  {
		    m_ROSNode->subscribe( m_OctreeTopic, m_OctreeMessage, &OctreeVisualizer::IncomingOctreeCallback, this, 10 );
		  }
	}

	void OctreeVisualizer::Unsubscribe()
	{
		if ( !m_OctreeTopic.empty() )
		  {
		    m_ROSNode->unsubscribe( m_OctreeTopic );
		  }
	}

	void OctreeVisualizer::Update( float dt )
	{
		m_OctreeMessage.lock();

		if ( m_NewMessage )
		{
			scan_utils::Octree<char> octree(0,0,0,0,0,0,1,0);
			octree.setFromMsg(m_OctreeMessage);

			std::list<scan_utils::Triangle> triangles;
			octree.getAllTriangles(triangles);

			std::list<scan_utils::Triangle>::iterator it;
			Ogre::Vector3 v1, v2, v3;

			m_ManualObject->clear();
			m_ManualObject->begin( m_MaterialName, Ogre::RenderOperation::OT_TRIANGLE_LIST );

			int i = 0;
			for ( it = triangles.begin(); it!=triangles.end(); it++, i += 3)
			{
				v1 = Ogre::Vector3( it->p1.x, it->p1.y, it->p1.z );
				RobotToOgre( v1 );
				m_ManualObject->position( v1 );
				m_ManualObject->colour( m_R, m_G, m_B );
				v2 = Ogre::Vector3( it->p2.x, it->p2.y, it->p2.z );
				RobotToOgre( v2 );
				m_ManualObject->position( v2 );
				m_ManualObject->colour( m_R, m_G, m_B );
				v3 = Ogre::Vector3( it->p3.x, it->p3.y, it->p3.z );
				RobotToOgre( v3 );
				m_ManualObject->position( v3 );
				m_ManualObject->colour( m_R, m_G, m_B );

				//m_ManualObject->triangle( i, i + 2, i + 1 );
			}
			m_ManualObject->end();

			m_NewMessage = false;
		}

		m_OctreeMessage.unlock();
	}

	void OctreeVisualizer::IncomingOctreeCallback()
	{
		m_OctreeMessage.lock();
		m_NewMessage = true;
		m_OctreeMessage.unlock();
	}

	void OctreeVisualizer::SetOctreeTopic( const std::string& topic )
	{
		Unsubscribe();

		m_OctreeTopic = topic;

		Subscribe();
	}

	void OctreeVisualizer::SetColor( float r, float g, float b )
	{
	  m_R = r;
	  m_G = g;
	  m_B = b;
	}
}
