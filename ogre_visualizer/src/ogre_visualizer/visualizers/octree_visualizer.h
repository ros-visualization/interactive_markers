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
#include <octree.h>

#include <Ogre.h>

namespace ogre_vis
{

	class OctreeVisualizer : public VisualizerBase
	{
	public:
		OctreeVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled );
		virtual ~OctreeVisualizer();

		void setOctreeTopic( const std::string& topic );
		void setColor( float r, float g, float b );

		// Overrides from VisualizerBase
    virtual void fillPropertyGrid( wxPropertyGrid* property_grid );
    virtual void propertyChanged( wxPropertyGridEvent& event );

		virtual void update( float dt );

	protected:
		// overrides from VisualizerBase
		virtual void onEnable();
		virtual void onDisable();
		void subscribe();
		void unsubscribe();

		void incomingOctreeCallback();

		float r_;
		float g_;
		float b_;

		Ogre::SceneNode* scene_node_;
		Ogre::ManualObject* manual_object_;
		Ogre::MaterialPtr material_;
		std::string material_name_;
		std::string octree_topic_;

		scan_utils::OctreeMsg octree_message_;
		typedef std::vector<Ogre::Vector3> V_Vector3;
		V_Vector3 vertices_;
		V_Vector3 normals_;

		bool new_message_;
	};

}
#endif /* OCTREE_VISUALIZER_H_ */
