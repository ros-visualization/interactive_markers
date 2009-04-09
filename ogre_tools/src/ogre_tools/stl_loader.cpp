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

#include "stl_loader.h"
#include <ros/console.h>

namespace ogre_tools
{

STLLoader::STLLoader()
{

}

STLLoader::~STLLoader()
{

}

bool STLLoader::load(const std::string& path)
{
  FILE* input = fopen( path.c_str(), "r" );
  if ( !input )
  {
    ROS_ERROR( "Could not open '%s' for read\n", path.c_str() );
    return false;
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

  for ( unsigned int currentTriangle = 0; currentTriangle < numTriangles; ++currentTriangle )
  {
    Triangle tri;

    tri.normal_.x = *(float*)pos;
    pos += 4;
    tri.normal_.y = *(float*)pos;
    pos += 4;
    tri.normal_.z = *(float*)pos;
    pos += 4;

    tri.vertices_[0].x = *(float*)pos;
    pos += 4;
    tri.vertices_[0].y = *(float*)pos;
    pos += 4;
    tri.vertices_[0].z = *(float*)pos;
    pos += 4;

    tri.vertices_[1].x = *(float*)pos;
    pos += 4;
    tri.vertices_[1].y = *(float*)pos;
    pos += 4;
    tri.vertices_[1].z = *(float*)pos;
    pos += 4;

    tri.vertices_[2].x = *(float*)pos;
    pos += 4;
    tri.vertices_[2].y = *(float*)pos;
    pos += 4;
    tri.vertices_[2].z = *(float*)pos;
    pos += 4;

    // Blender was writing a large number into this short... am I misinterpreting what the attribute byte count is supposed to do?
    //unsigned short attributeByteCount = *(unsigned short*)pos;
    pos += 2;

    //pos += attributeByteCount;

    if (tri.normal_.squaredLength() < 0.001)
    {
      Ogre::Vector3 side1 = tri.vertices_[0] - tri.vertices_[1];
      Ogre::Vector3 side2 = tri.vertices_[1] - tri.vertices_[2];
      tri.normal_ = side1.crossProduct(side2);
      tri.normal_.normalise();
    }

    triangles_.push_back(tri);
  }

  return true;
}

}
