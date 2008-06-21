//#include "ILModel.h"
#include <irrlicht.h>

class ILModel{
public:
	irr::scene::IAnimatedMesh *mesh;
	irr::scene::IAnimatedMeshSceneNode *node;
	irr::scene::ISceneManager *manager;
	
ILModel()
{
	mesh = 0;
	node = 0;
	manager = 0;
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName, bool displayNow)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
	if(displayNow)
		node = manager->addAnimatedMeshSceneNode(mesh);
}

ILModel(irr::scene::ISceneManager *mngr, irr::c8 *fName, irr::core::vector3d<irr::f32> position)
{
	manager = mngr;
	mesh = mngr->getMesh(fName);
	node = 0;
	node = manager->addAnimatedMeshSceneNode(mesh);
	node->setPosition(rightToLeft(position));
}

~ILModel()
{
	kill();
}
	
//you may want to lock the rendere before using and unlock after using me
void draw()
{
	if(!node)
		node = manager->addAnimatedMeshSceneNode(mesh);
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
		node->setRotation(rightToLeft(rotation));
	}
}

//lock renderer before using and unlock after using
void kill()
{
	if(node)
		node->remove();
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
	return right;//make sure you kill me later
}

irr::core::vector3d<irr::f32> rightToLeft(irr::core::vector3d<irr::f32> right)
{
	irr::core::vector3d<irr::f32> left(-right.Y,right.Z,right.X);
	return left;//make sure you kill me later
}
};