#include "initialization.h"

#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <OgreLogManager.h>

#include <exception>
#include <stdexcept>

#include <ros/common.h>

namespace ogre_tools
{
void initializeOgre()
{
  try
  {
    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog( "Ogre.log", false, false, false );

    std::string plugin_cfg = "/etc/OGRE/plugins.cfg";
    bool has_plugin_cfg = false;
#ifdef HAS_INSTALLED_OGRE
    has_plugin_cfg = true;
#else
    plugin_cfg = "";
#endif

    Ogre::Root* root = new Ogre::Root( plugin_cfg );
    if ( !has_plugin_cfg )
    {
      root->loadPlugin( "RenderSystem_GL" );
      root->loadPlugin( "Plugin_OctreeSceneManager" );
      root->loadPlugin( "Plugin_ParticleFX" );
    }

    // Taken from gazebo
    Ogre::RenderSystemList *rsList = root->getAvailableRenderers();

    Ogre::RenderSystem* render_system = NULL;
    Ogre::RenderSystemList::iterator renderIt = rsList->begin();
    Ogre::RenderSystemList::iterator renderEnd = rsList->end();
    for ( ; renderIt != renderEnd; ++renderIt )
    {
      render_system = *renderIt;

      if ( render_system->getName() == "OpenGL Rendering Subsystem" )
      {
        break;
      }
    }

    if ( render_system == NULL )
    {
      throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
    }

    render_system->setConfigOption("Full Screen","No");
    render_system->setConfigOption("FSAA","2");
    render_system->setConfigOption("RTT Preferred Mode", "PBuffer");

    root->setRenderSystem( render_system );

    root->initialise( false );

    std::string ogre_tools_path = ros::get_package_path(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/models", "FileSystem", ROS_PACKAGE_NAME );
  }
  catch ( Ogre::Exception& e )
  {
      printf( "Failed to initialize Ogre: %s\n", e.what() );
      throw;
  }
}

void cleanupOgre()
{
  delete Ogre::Root::getSingletonPtr();
}

void initializeResources( const V_string& resource_paths )
{
  V_string::const_iterator path_it = resource_paths.begin();
  V_string::const_iterator path_end = resource_paths.end();
  for( ; path_it != path_end; ++path_it )
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *path_it, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  }

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

} // namespace ogre_tools
