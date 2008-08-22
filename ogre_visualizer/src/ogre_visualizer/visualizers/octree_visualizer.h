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
 * octree_visualizer.h
 *
 *  Created on: Aug 20, 2008
 *      Author: Matthew Piccoli and Matei Ciocarlie
 */

#ifndef OCTREE_VISUALIZER_H_
#define OCTREE_VISUALIZER_H_

#include "../visualizer_base.h"
#include <scan_utils/OctreeMsg.h>

#include <Ogre.h>
namespace ogre_vis
{

	class OctreeVisualizer : public VisualizerBase
	{
	public:
		OctreeVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled );
		virtual ~OctreeVisualizer();

		void SetOctreeTopic( const std::string& topic );
		void SetColor( float r, float g, float b );

		virtual void Update( float dt );

	protected:
		// overrides from VisualizerBase
		virtual void OnEnable();
		virtual void OnDisable();
		void Subscribe();
		void Unsubscribe();

		void IncomingOctreeCallback();

		float m_R;
		float m_G;
		float m_B;

		Ogre::SceneNode* m_SceneNode;
		Ogre::ManualObject* m_ManualObject;
		Ogre::MaterialPtr m_Material;
		std::string m_MaterialName;
		std::string m_OctreeTopic;

		scan_utils::OctreeMsg m_OctreeMessage;

		bool m_NewMessage;
	};

}
#endif /* OCTREE_VISUALIZER_H_ */
