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
 * Show a right handed coordinate system in irrlicht's left handed environment
 *
 * Written by Matthew Piccoli
 */

#include <irrlicht.h>

///Irrlicht class that automatically generates a universal coordinate system
class ILUCS
{
public:
	irr::scene::IAnimatedMeshSceneNode *x;
	irr::scene::IAnimatedMeshSceneNode *y;
	irr::scene::IAnimatedMeshSceneNode *z;
	irr::scene::IAnimatedMesh *Mesh;
	//irr::scene::IAnimatedMesh *yMesh;
	//irr::scene::IAnimatedMesh *zMesh;
	irr::scene::ISceneManager *SceneManager;
	bool leftHandedToRight;
	irr::video::SMaterial m_material;
	irr::s32 startId;

	///Constructor, generates the mesh used
	ILUCS(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, bool leftToRight, irr::s32 startID)
	{
		startId = startID;
		x = NULL;
		y = NULL;
		z = NULL;
		SceneManager = mgr;
		leftHandedToRight = leftToRight;
		m_material.Lighting = false;
		m_material.Wireframe = false;
		m_material.PointCloud = false;
		m_material.BackfaceCulling = false;
		m_material.Thickness = 1;
		Mesh = SceneManager->addArrowMesh("UCSArrow",irr::video::SColor::SColor(255,0,0,0),irr::video::SColor::SColor(255,0,0,0));
		//yMesh = SceneManager->addArrowMesh("yUCS",irr::video::SColor::SColor(255,0,255,0),irr::video::SColor::SColor(255,0,255,0));
		//zMesh = SceneManager->addArrowMesh("zUCS",irr::video::SColor::SColor(255,0,0,255),irr::video::SColor::SColor(255,0,0,255));
	}

	///Shows/disables UCS.  Left handed to left handed not implemented.
	void setVisible(bool visible)
	{
		if(visible)
		{
			SceneManager->getVideoDriver()->setMaterial(m_material);
			if(leftHandedToRight)
			{

				x = SceneManager->addAnimatedMeshSceneNode(Mesh);
				//x->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				//x->setRotation(irr::core::vector3d<irr::f32>(180,0,0));
				x->setRotation(irr::core::vector3d<irr::f32>(90,0,0));
				x->getMaterial(0).AmbientColor.set(255,255,0,0);
				x->getMaterial(1).AmbientColor.set(255,255,0,0);


				y = SceneManager->addAnimatedMeshSceneNode(Mesh);
				//y->setRotation(irr::core::vector3d<irr::f32>(90,0,90));
				y->setRotation(irr::core::vector3d<irr::f32>(0,0,90));
				//y->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				y->getMaterial(0).AmbientColor.set(255,0,255,0);
				y->getMaterial(1).AmbientColor.set(255,0,255,0);

				z = SceneManager->addAnimatedMeshSceneNode(Mesh);
				//z->setRotation(irr::core::vector3d<irr::f32>(0,0,-90));
				//z->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				z->getMaterial(0).AmbientColor.set(255,0,0,255);
				z->getMaterial(1).AmbientColor.set(255,0,0,255);

			}
			else
			{
				//are these rotations right???
				/*x = SceneManager->addAnimatedMeshSceneNode(xMesh);
				x->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				y = SceneManager->addAnimatedMeshSceneNode(yMesh);
				y->setRotation(irr::core::vector3d<irr::f32>(0,0,90));
				y->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				z = SceneManager->addAnimatedMeshSceneNode(zMesh);
				z->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				y->setRotation(irr::core::vector3d<irr::f32>(0,-90,0));*/
			}
			if(!startId < 0)
			{
				x->setID(startId);
				y->setID(startId+1);
				z->setID(startId+2);
			}
		}
		else if(x != NULL)
		{
			x->remove();
			y->remove();
			z->remove();
			x = NULL;
			y = NULL;
			z = NULL;
		}
	}
	///Exists for compatibility only
	irr::u32 getMaterialCount() {
		return 1;
	}
	///returns material that was generated in constructor
	irr::video::SMaterial& getMaterial(irr::u32 i) {
		return m_material;
	}
	///destructor
	~ILUCS()
	{
		setVisible(false);
		SceneManager->getMeshCache()->removeMesh(Mesh);
		//SceneManager->getMeshCache()->removeMesh(yMesh);
		//SceneManager->getMeshCache()->removeMesh(zMesh);
	}
};
