#include <OGRE/OgreRoot.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSkeletonManager.h>
#include <OGRE/OgreMeshSerializer.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreMath.h>
#include <OGRE/OgreDefaultHardwareBufferManager.h>
#include <OGRE/OgreManualObject.h>

#include "ogre_tools/stl_loader.h"

#include <ros/console.h>

/**
 * @file
 *
 * The stl_to_mesh binary converts a binary STL file to an Ogre mesh.  See http://en.wikipedia.org/wiki/STL_(file_format)#Binary_STL for a description of
 * the file format.
 *
 * @par Usage
 @verbatim
 $ stl_to_mesh <stl files> <output directory>
 $ stl_to_mesh <stl file> <output file>
 @endverbatim
 * See http://pr.willowgarage.com/wiki/STL_To_Ogre_Mesh_Converter for more information
 */

using namespace Ogre;
using namespace ogre_tools;

void calculateUV(const Ogre::Vector3& vec, float& u, float& v)
{
  Ogre::Vector3 pos(vec);
  pos.normalise();
  u = acos( pos.y / pos.length() );

  float val = pos.x / ( sin( u ) );
  v = acos( val );

  u /= Ogre::Math::PI;
  v /= Ogre::Math::PI;
}

int main( int argc, char** argv )
{
  if ( argc < 3 )
  {
    ROS_INFO( "Usage: stl_to_mesh <stl files> <output directory>" );
    ROS_INFO( "or     stl_to_mesh <stl file> <output file>" );

    return 0;
  }

  typedef std::vector<std::string> V_string;
  V_string inputFiles;
  V_string outputFiles;
  std::string outputDirectory = argv[ argc - 1 ];
  if ( outputDirectory.rfind( ".mesh" ) != std::string::npos )
  {
    ROS_INFO( "Converting single mesh: %s to %s", argv[1], outputDirectory.c_str() );
    inputFiles.push_back( argv[ 1 ] );
    outputFiles.push_back( outputDirectory );
  }
  else
  {
    ROS_INFO( "Converting multiple meshes, into output directory %s...", outputDirectory.c_str() );

    for ( int i = 1; i < argc - 1; ++i )
    {
      std::string inputFile = argv[ i ];
      inputFiles.push_back( inputFile );

      // convert the input filename to the mesh file
      size_t pos = inputFile.rfind( ".stl" );
      if ( pos == std::string::npos )
      {
        pos = inputFile.rfind( ".STL" );
      }

      if ( pos == std::string::npos )
      {
        ROS_INFO( "Input file %s is not a .stl or .STL file!", inputFile.c_str() );
        exit(1);
      }

      std::string outputFile = inputFile;
      outputFile.replace( pos, strlen( ".stl" ), ".mesh" );

      pos = outputFile.rfind( "/" );
      if ( pos != std::string::npos )
      {
        outputFile.erase( 0, pos );
      }

      outputFile = outputDirectory + "/" + outputFile;

      outputFiles.push_back( outputFile );
    }
  }

  // NB some of these are not directly used, but are required to
  //   instantiate the singletons used in the dlls
  LogManager* logMgr = 0;
  Math* mth = 0;
  MaterialManager* matMgr = 0;
  SkeletonManager* skelMgr = 0;
  MeshSerializer* meshSerializer = 0;
  DefaultHardwareBufferManager *bufferManager = 0;
  MeshManager* meshMgr = 0;
  ResourceGroupManager* rgm = 0;

  try
  {
    logMgr = new LogManager();
    logMgr->createLog( "STLToMesh_Ogre.log", false, false, true );
    rgm = new ResourceGroupManager();
    mth = new Math();
    meshMgr = new MeshManager();
    matMgr = new MaterialManager();
    matMgr->initialise();
    skelMgr = new SkeletonManager();
    meshSerializer = new MeshSerializer();
    bufferManager = new DefaultHardwareBufferManager(); // needed because we don't have a rendersystem

    for ( size_t i = 0; i < inputFiles.size(); ++i )
    {
      std::string inputFile = inputFiles[ i ];
      std::string outputFile = outputFiles[ i ];

      STLLoader loader;
      if (!loader.load(inputFile))
      {
        exit( 1 );
      }

      ROS_INFO( "Converting %s to %s...", inputFile.c_str(), outputFile.c_str() );
      ROS_INFO( "%d triangles", loader.triangles_.size() );

      ManualObject* object = new ManualObject( "the one and only" );
      object->begin( "BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST );

      unsigned int vertexCount = 0;
      STLLoader::V_Triangle::const_iterator it = loader.triangles_.begin();
      STLLoader::V_Triangle::const_iterator end = loader.triangles_.end();
      for (; it != end; ++it )
      {
        const STLLoader::Triangle& tri = *it;

        float u, v;
        u = v = 0.0f;
        object->position( tri.vertices_[0] );
        object->normal( tri.normal_);
        calculateUV( tri.vertices_[0], u, v );
        object->textureCoord( u, v );

        object->position( tri.vertices_[1] );
        object->normal( tri.normal_);
        calculateUV( tri.vertices_[1], u, v );
        object->textureCoord( u, v );

        object->position( tri.vertices_[2] );
        object->normal( tri.normal_);
        calculateUV( tri.vertices_[2], u, v );
        object->textureCoord( u, v );

        object->triangle( vertexCount + 0, vertexCount + 1, vertexCount + 2 );

        vertexCount += 3;
      }

      object->end();

      std::stringstream ss;
      ss << "converted" << i;
      MeshPtr mesh = object->convertToMesh( ss.str(), ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
      meshSerializer->exportMesh( mesh.get(), outputFile, Serializer::ENDIAN_LITTLE );
    }
  }
  catch ( Exception& e )
  {
    ROS_ERROR( "%s", e.what() );
  }

  delete meshSerializer;
  delete skelMgr;
  delete matMgr;
  delete meshMgr;
  delete bufferManager;
  delete mth;
  delete rgm;
  delete logMgr;

  return 0;
}
