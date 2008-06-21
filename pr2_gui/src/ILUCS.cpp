#include <irrlicht.h>

class ILUCS
{
public:
	irr::scene::IAnimatedMeshSceneNode *x;
	irr::scene::IAnimatedMeshSceneNode *y;
	irr::scene::IAnimatedMeshSceneNode *z;
	irr::scene::IAnimatedMesh *xMesh;
	irr::scene::IAnimatedMesh *yMesh;
	irr::scene::IAnimatedMesh *zMesh;
	irr::scene::ISceneManager *manager;
	
	ILUCS(irr::scene::ISceneManager *mngr, bool leftHanded)
	{
		manager = mngr;
		if(leftHanded)
		{
			xMesh = manager->addArrowMesh("xUCS",irr::video::SColor::SColor(255,255,0,0),irr::video::SColor::SColor(255,255,0,0));
			x = manager->addAnimatedMeshSceneNode(xMesh);
			x->setMaterialFlag(irr::video::EMF_LIGHTING,false);
			x->setRotation(irr::core::vector3d<irr::f32>(90,0,0));
			yMesh = manager->addArrowMesh("yUCS",irr::video::SColor::SColor(255,0,255,0),irr::video::SColor::SColor(255,0,255,0));
			y = manager->addAnimatedMeshSceneNode(yMesh);
			y->setRotation(irr::core::vector3d<irr::f32>(0,0,90));
			y->setMaterialFlag(irr::video::EMF_LIGHTING,false);
			zMesh = manager->addArrowMesh("zUCS",irr::video::SColor::SColor(255,0,0,255),irr::video::SColor::SColor(255,0,0,255));
			z = manager->addAnimatedMeshSceneNode(zMesh);
			z->setMaterialFlag(irr::video::EMF_LIGHTING,false);
			
		}
		else
		{
			//x = new irr::core::line3d(0,0,0,1,0,0);
			//y = new irr::core::line3d(0,0,0,0,1,0);
			//z = new irr::core::line3d(0,0,0,0,0,1);
		}
	}
	
	~ILUCS()
	{
		x->remove();
		y->remove();
		z->remove();
		manager->getMeshCache()->removeMesh(xMesh);
		manager->getMeshCache()->removeMesh(yMesh);
		manager->getMeshCache()->removeMesh(zMesh);
	}
};