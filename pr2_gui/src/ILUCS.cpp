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
	irr::scene::ISceneManager *SceneManager;
	bool leftHanded;
	irr::video::SMaterial m_material;
	
	
	ILUCS(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, bool leftH) 
	{
		SceneManager = mgr;
		std::cout << "constructing\n";
		leftHanded = leftH;
		m_material.Lighting = false;
		m_material.Wireframe = false;
		m_material.PointCloud = false;
		m_material.BackfaceCulling = false;
		m_material.Thickness = 1;
		xMesh = SceneManager->addArrowMesh("xUCS",irr::video::SColor::SColor(255,255,0,0),irr::video::SColor::SColor(255,255,0,0));
		yMesh = SceneManager->addArrowMesh("yUCS",irr::video::SColor::SColor(255,0,255,0),irr::video::SColor::SColor(255,0,255,0));
		zMesh = SceneManager->addArrowMesh("zUCS",irr::video::SColor::SColor(255,0,0,255),irr::video::SColor::SColor(255,0,0,255));
	}
	
	void setVisible(bool visible)
	{
		if(visible)
		{
			SceneManager->getVideoDriver()->setMaterial(m_material);
			if(leftHanded)
			{
				
				x = SceneManager->addAnimatedMeshSceneNode(xMesh);
				x->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				x->setRotation(irr::core::vector3d<irr::f32>(90,0,0));
				
				y = SceneManager->addAnimatedMeshSceneNode(yMesh);
				y->setRotation(irr::core::vector3d<irr::f32>(0,0,90));
				y->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				
				z = SceneManager->addAnimatedMeshSceneNode(zMesh);
				z->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				
			}
			else
			{
				//are these rotations right???
				x = SceneManager->addAnimatedMeshSceneNode(xMesh);
				x->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				y = SceneManager->addAnimatedMeshSceneNode(yMesh);
				y->setRotation(irr::core::vector3d<irr::f32>(0,0,90));
				y->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				z = SceneManager->addAnimatedMeshSceneNode(zMesh);
				z->setMaterialFlag(irr::video::EMF_LIGHTING,false);
				y->setRotation(irr::core::vector3d<irr::f32>(0,-90,0));
			}
		}
		else
		{
			x->remove();
			y->remove();
			z->remove();
		}
	}

	irr::u32 getMaterialCount() {
		return 1;
	}

	irr::video::SMaterial& getMaterial(irr::u32 i) {
		return m_material;
	}	

	~ILUCS()
	{
		setVisible(false);
		SceneManager->getMeshCache()->removeMesh(xMesh);
		SceneManager->getMeshCache()->removeMesh(yMesh);
		SceneManager->getMeshCache()->removeMesh(zMesh);
	}
};