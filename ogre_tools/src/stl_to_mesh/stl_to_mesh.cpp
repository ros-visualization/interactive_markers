#include <OgreRoot.h>
#include <OgreLogManager.h>
#include <OgreMaterialManager.h>
#include <OgreSkeletonManager.h>
#include <OgreMeshSerializer.h>
#include <OgreMeshManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreMath.h>
#include <OgreDefaultHardwareBufferManager.h>
#include <OgreManualObject.h>

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

void calculateUV(float x, float y, float z, float& u, float& v)
{
  Ogre::Vector3 pos(x,y,z);
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
    printf( "Usage: stl_to_mesh <stl files> <output directory>\n" );
    printf( "or     stl_to_mesh <stl file> <output file>\n" );

    return 0;
  }

  typedef std::vector<std::string> V_string;
  V_string inputFiles;
  V_string outputFiles;
  std::string outputDirectory = argv[ argc - 1 ];
  if ( outputDirectory.rfind( ".mesh" ) != std::string::npos )
  {
    printf( "Converting single mesh: %s to %s\n", argv[1], outputDirectory.c_str() );
    inputFiles.push_back( argv[ 1 ] );
    outputFiles.push_back( outputDirectory );
  }
  else
  {
    printf( "Converting multiple meshes, into output directory %s...\n", outputDirectory.c_str() );

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
        printf( "Input file %s is not a .stl or .STL file!\n", inputFile.c_str() );
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

      printf( "Converting %s to %s...\n", inputFile.c_str(), outputFile.c_str() );

      FILE* input = fopen( inputFile.c_str(), "r" );
      if ( !input )
      {
        printf( "Could not open '%s' for read\n", inputFile.c_str() );
        exit( 1 );
      }

      /* from wikipedia:
       * Because ASCII STL files can become very large, a binary version of STL exists. A binary STL file has an 80 character header
       * (which is generally ignored - but which should never begin with 'solid' because that will lead most software to assume that
       * this is an ASCII STL file). Following the header is a 4 byte unsigned integer indicating the number of triangular facets in
       * the file. Following that is data describing each triangle in turn. The file simply ends after the last triangle.
       *
       * Each triangle is described by twelve 32-bit-floating point numbers: three for the normal and then three for the X/Y/Z coordinate
       * of each vertex - just as with the ASCII version of STL. After the twelve floats there is a two byte unsigned 'short' integer that
       * is the 'attribute byte count' - in the standard format, this should be zero because most software does not understand anything else.
       *
       * Floating point numbers are represented as IEEE floating point numbers and the endianness is assumed to be little endian although this
       * is not stated in documentation.
       */

      // find the file size
      fseek( input, 0, SEEK_END );
      long long fileSize = ftell( input );
      fseek( input, 0, SEEK_SET );

      char* buffer = new char[ fileSize ];
      fread( buffer, fileSize, 1, input );

      fclose( input );

      char* pos = buffer;
      pos += 80; // skip the 80 byte header

      unsigned int numTriangles = *(unsigned int*)pos;
      pos += 4;

      printf( "%d triangles\n", numTriangles );

      ManualObject* object = new ManualObject( "the one and only" );
      object->begin( "BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST );

      unsigned int vertexCount = 0;
      for ( unsigned int currentTriangle = 0; currentTriangle < numTriangles; ++currentTriangle )
      {
        float nX = *(float*)pos;
        pos += 4;
        float nY = *(float*)pos;
        pos += 4;
        float nZ = *(float*)pos;
        pos += 4;

        float v1X = *(float*)pos;
        pos += 4;
        float v1Y = *(float*)pos;
        pos += 4;
        float v1Z = *(float*)pos;
        pos += 4;

        float v2X = *(float*)pos;
        pos += 4;
        float v2Y = *(float*)pos;
        pos += 4;
        float v2Z = *(float*)pos;
        pos += 4;

        float v3X = *(float*)pos;
        pos += 4;
        float v3Y = *(float*)pos;
        pos += 4;
        float v3Z = *(float*)pos;
        pos += 4;

        unsigned short attributeByteCount = *(unsigned short*)pos;
        pos += 2;

        pos += attributeByteCount;

        float u, v;

        object->position( v1X, v1Y, v1Z );
        object->normal( nX, nY, nZ );
        calculateUV( v1X, v1Y, v1Z, u, v );
        object->textureCoord( u, v );

        object->position( v2X, v2Y, v2Z );
        object->normal( nX, nY, nZ );
        calculateUV( v2X, v2Y, v2Z, u, v );
        object->textureCoord( u, v );

        object->position( v3X, v3Y, v3Z );
        object->normal( nX, nY, nZ );
        calculateUV( v3X, v3Y, v3Z, u, v );
        object->textureCoord( u, v );

        object->triangle( vertexCount + 0, vertexCount + 1, vertexCount + 2 );

        vertexCount += 3;
      }

      object->end();

      delete [] buffer;

      std::stringstream ss;
      ss << "converted" << i;
      MeshPtr mesh = object->convertToMesh( ss.str(), ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
      meshSerializer->exportMesh( mesh.get(), outputFile, Serializer::ENDIAN_LITTLE );
    }
  }
  catch ( Exception& e )
  {
    printf( "Error: %s\n", e.what() );
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
