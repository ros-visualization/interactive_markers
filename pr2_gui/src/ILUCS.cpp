#include <irrlicht.h>

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
	bool leftHanded;
	irr::video::SMaterial m_material;
	irr::s32 startId;
	
	
	ILUCS(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, bool leftH, irr::s32 startID) 
	{
		startId = startID;
		x = NULL;
		y = NULL;
		z = NULL;
		SceneManager = mgr;
		leftHanded = leftH;
		m_material.Lighting = false;
		m_material.Wireframe = false;
		m_material.PointCloud = false;
		m_material.BackfaceCulling = false;
		m_material.Thickness = 1;
		Mesh = SceneManager->addArrowMesh("UCSArrow",irr::video::SColor::SColor(255,0,0,0),irr::video::SColor::SColor(255,0,0,0));
		//yMesh = SceneManager->addArrowMesh("yUCS",irr::video::SColor::SColor(255,0,255,0),irr::video::SColor::SColor(255,0,255,0));
		//zMesh = SceneManager->addArrowMesh("zUCS",irr::video::SColor::SColor(255,0,0,255),irr::video::SColor::SColor(255,0,0,255));
	}
	
	void setVisible(bool visible)
	{
		if(visible)
		{
			SceneManager->getVideoDriver()->setMaterial(m_material);
			if(leftHanded)
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

	irr::u32 getMaterialCount() {
		return 1;
	}

	irr::video::SMaterial& getMaterial(irr::u32 i) {
		return m_material;
	}	

	~ILUCS()
	{
		setVisible(false);
		SceneManager->getMeshCache()->removeMesh(Mesh);
		//SceneManager->getMeshCache()->removeMesh(yMesh);
		//SceneManager->getMeshCache()->removeMesh(zMesh);
	}
};
