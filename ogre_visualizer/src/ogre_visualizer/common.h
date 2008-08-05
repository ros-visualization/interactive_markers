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

// DO NOT INCLUDE THIS FROM WITHIN ANOTHER HEADER, AS IT WILL PULL IN ALL OF OGRE

#ifndef OGRE_VISUALIZER_COMMON_H
#define OGRE_VISUALIZER_COMMON_H

#include <Ogre.h>

namespace ogre_vis
{

extern Ogre::Matrix3 g_OgreToRobotMatrix;
extern Ogre::Matrix3 g_RobotToOgreMatrix;

extern Ogre::Quaternion g_OgreToRobotQuat;
extern Ogre::Quaternion g_RobotToOgreQuat;

void InitializeCommon();

inline void RobotToOgre( Ogre::Vector3& point )
{
  point = g_RobotToOgreMatrix * point;
}

inline void RobotToOgre( Ogre::Quaternion& quat )
{
  quat = g_RobotToOgreQuat * quat;
}

inline void RobotToOgre( Ogre::Matrix3& mat )
{
  mat = g_RobotToOgreMatrix * mat;
}



inline void OgreToRobot( Ogre::Vector3& point )
{
  point = g_OgreToRobotMatrix * point;
}

inline void OgreToRobot( Ogre::Quaternion& quat )
{
  quat = g_OgreToRobotQuat * quat;
}

inline void OgreToRobot( Ogre::Matrix3& mat )
{
  mat = g_OgreToRobotMatrix * mat;
}


inline Ogre::Matrix3 OgreMatrixFromRobotEulers( float yaw, float pitch, float roll )
{
  Ogre::Matrix3 mat;
  mat.FromEulerAnglesZYX( Ogre::Radian( yaw ), Ogre::Radian( pitch ), Ogre::Radian( roll ) );
  RobotToOgre( mat );

  return mat;
}

}
#endif
