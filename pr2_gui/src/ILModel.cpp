/*Irrlicht model class wrapper for left handed coordinate systems*/

//#include "ILModel.h"
#include <irrlicht.h>

class ILModel{
public:
	irr::scene::IAnimatedMesh *mesh;
	irr::scene::IAnimatedMeshSceneNode *node;
	irr::scene::ISceneManager *manager;
	float defaultXRotation;
	float defaultYRotation;
	float defaultZRotation;
	
ILModel()
{
	mesh = 0;
	node = 0;
	manager = 0;
	defaultXRotation = 0.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = -90.0f;
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
	defaultXRotation = 0.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = -90.0f;
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName, bool displayNow)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
	defaultXRotation = 0.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = -90.0f;
	if(displayNow)
	{
		node = manager->addAnimatedMeshSceneNode(mesh);
		rotateTo(irr::core::vector3d<irr::f32>(0.0f,0.0f,0.0f));
	}
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName, irr::core::vector3d<irr::f32> position, irr::core::vector3d<irr::f32> rotation)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
	defaultXRotation = 0.0f;
	defaultYRotation = 0.0f;
	defaultZRotation = -90.0f;
	node = manager->addAnimatedMeshSceneNode(mesh);
	node->setPosition(rightToLeft(position));
	rotateTo(rotation);
}

~ILModel()
{
	kill();
}
	
//you may want to lock the rendere before using and unlock after using me
void draw()
{
	if(!node)
	{
		node = manager->addAnimatedMeshSceneNode(mesh);
		rotateTo(irr::core::vector3d<irr::f32>(0.0f,0.0f,0.0f));
	}
}

void goTo(irr::core::vector3d<irr::f32> position)
{
	if(node)
	{
		node->setPosition(rightToLeft(position));
	}
}

void rotateTo(irr::core::vector3d<irr::f32> rotation)
{
	if(node)
	{
		//do I want this from left to right too???  Assuming yes.
		node->setRotation(rightToLeft(rotation) + rightToLeft(irr::core::vector3d<irr::f32>(defaultXRotation,defaultYRotation,defaultZRotation)));
	}
}

//lock renderer before using and unlock after using
void kill()
{
	if(node)
	{
		node->remove();
	}
}

irr::scene::ISceneManager* getManager()
{
	return manager;
}

irr::scene::IAnimatedMesh* getMesh()
{
	return mesh;
}
//use me to know if active as well!
irr::scene::IAnimatedMeshSceneNode* getNode()
{
	return node;
}

irr::core::vector3d<irr::f32> leftToRight(irr::core::vector3d<irr::f32> left)
{
	irr::core::vector3d<irr::f32> right(left.Z,-left.X,left.Y);
	return right;
}

irr::core::vector3d<irr::f32> rightToLeft(irr::core::vector3d<irr::f32> right)
{
	irr::core::vector3d<irr::f32> left(-right.Y,right.Z,right.X);
	return left;
}
};