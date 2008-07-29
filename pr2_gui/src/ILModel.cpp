/*Irrlicht model class wrapper for left handed coordinate systems*/

//#include "ILModel.h"
#include <irrlicht.h>

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

~ILModel()
{
	kill();
}

void setParent(irr::scene::ISceneNode *prnt)
{
	parent = prnt;
	if(node)
		node->setParent(parent);
}

void setPosition(float x, float y, float z)
{
	if(node)
	{
		node->setPosition(irr::core::vector3d<irr::f32>(-x,-y,-z));
	}
}

//Roll, Pitch, Yaw
void setRotation(float roll, float pitch, float yaw)
{
	if(node)
	{
		node->setRotation(irr::core::vector3d<irr::f32>(-roll *180 / 3.14159, -pitch *180 / 3.14159, yaw *180 / 3.14159) + irr::core::vector3d<irr::f32>(defaultXRotation,defaultYRotation,defaultZRotation));
	}
}

//lock renderer before using and unlock after using
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

irr::scene::ISceneManager* getManager()
{
	return manager;
}

irr::scene::IAnimatedMesh* getMesh()
{
	return mesh;
}

//use me to know if active as well!
irr::scene::ISceneNode* getNode()
{
	return node;
}

irr::s32 getID()
{
	return ID;
}

void setScale(float xScale, float yScale, float zScale)
{
	if(node)
	{
		node->setScale(irr::core::vector3d<irr::f32>(xScale,zScale,yScale));//yes, this is supposed to be x z y
	}
}

void setVisible(bool visible)
{
	if(node)
		node->setVisible(visible);
}
};
