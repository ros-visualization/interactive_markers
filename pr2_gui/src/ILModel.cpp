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
 * Use models from right handed coordinate systems in a irrlicht's left handed environment.
 *
 * Written by Matthew Piccoli
 */

#include <irrlicht.h>

///Irrlicht model class wrapper for left handed coordinate systems
class ILModel{
public:
	irr::scene::IAnimatedMesh *mesh;
	irr::scene::ISceneNode *node;
	irr::scene::ISceneManager *manager;
	irr::scene::ISceneNode *parent;
	float defaultXRotation;
	float defaultYRotation;
	float defaultZRotation;
	irr::s32 ID;

///Constructor, takes a filename and loads the mesh/node
ILModel(irr::scene::ISceneManager *mngr, irr::scene::ISceneNode *prnt, irr::c8 *fName, irr::s32 id, float x, float y, float z, float roll, float pitch, float yaw)
{
	parent = prnt;
	mesh = 0;
	node = 0;
	defaultXRotation = -90.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = 180.0f;
	manager = mngr;
	ID = id;
	if(fName[0] != '\0')
	{
		mesh = mngr->getMesh(fName);
		if(mesh!=NULL)
		{
			node = manager->addAnimatedMeshSceneNode(mesh,parent,ID);
			setPosition(x,y,z);
			setRotation(roll,pitch,yaw);
		}
	}
}
///Constructor, takes in a node
ILModel(irr::scene::ISceneManager *mngr, irr::scene::ISceneNode *prnt, irr::scene::ISceneNode *inNode, irr::s32 id, float x, float y, float z, float roll, float pitch, float yaw)
{
	parent = prnt;
	mesh = 0;
	node = inNode;
	manager = mngr;
	defaultXRotation = -90.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = 180.0f;
	ID = id;
	if(node)
	{
		node->setParent(parent);
		node->setID(id);
		setPosition(x,y,z);
		setRotation(roll,pitch,yaw);
	}
}
///destructor
~ILModel()
{
	kill();
}
///changes the parent of the node
void setParent(irr::scene::ISceneNode *prnt)
{
	parent = prnt;
	if(node)
		node->setParent(parent);
}
///sets the position of the node
void setPosition(float x, float y, float z)
{
	if(node)
	{
		node->setPosition(irr::core::vector3d<irr::f32>(-x,-y,-z));
	}
}

///sets the rotation of the node in roll, pitch, yaw
void setRotation(float roll, float pitch, float yaw)
{
	if(node)
	{
		node->setRotation(irr::core::vector3d<irr::f32>(-roll *180 / 3.14159, -pitch *180 / 3.14159, yaw *180 / 3.14159) + irr::core::vector3d<irr::f32>(defaultXRotation,defaultYRotation,defaultZRotation));
	}
}

///removes the node, lock renderer before using and unlock after using
void kill()
{
	//std::cout << "trying to remove\n";
	if(node)
	{
		//std::cout << "removing model\n";
		node->remove();
		node = NULL;
		//std::cout << "removed model\n";
	}
}
///gets the ISceneManager to which the node belongs
irr::scene::ISceneManager* getManager()
{
	return manager;
}
///gets the mesh used by the node
irr::scene::IAnimatedMesh* getMesh()
{
	return mesh;
}

///gets the node, can be used to see if the model is in existance
irr::scene::ISceneNode* getNode()
{
	return node;
}
///returns the id, -1 if no id
irr::s32 getID()
{
	return ID;
}
///sets the scale of the model in each direction
void setScale(float xScale, float yScale, float zScale)
{
	if(node)
	{
		node->setScale(irr::core::vector3d<irr::f32>(xScale,zScale,yScale));//yes, this is supposed to be x z y
	}
}
///sets visibility of the node
void setVisible(bool visible)
{
	if(node)
		node->setVisible(visible);
}
};
