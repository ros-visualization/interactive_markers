#ifndef VIS3D_H
#define VIS3D_H

///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2008, Willow Garage Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/**
@mainpage

@b Vis3d is a 3D visualization of the robot's current state and sensor feedback using the Irrlicht rendering engine.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cloud"/std_msgs::PointCloudFloat32 : Point cloud received from head Hokuyo
- @b "scan"/std_msgs::LaserScan : Laser cloud received from base Hokuyo
- @b "cloudStereo"/std_msgs::PointCloudFloat32 : Point cloud received from stereo vision (type may be changed soon)
- @b "shutter"/std_msgs::Empty : Cue to erase "cloud" information
- @b "shutterScan"/std_msgs::Empty : Cue to erase "cloudFloor" information
- @b "shutterStereo"/std_msgs::Empty : Cue to erase "cloudStereo" information
- @b "visualizationMarker"/std_msgs::VisualizationMarker : User defined object in the 3d world
- @b "transform"/std_msgs::Empty : Cue to update model (new transform is available)

@todo
- Please put some stuff here
**/

#define intensityRange 16.0
#include <rosthread/member_thread.h>
#include "ILClient.hh"
#include "ILRender.hh"
#include "CustomNodes/ILPointCloud.hh"
#include "CustomNodes/ILGrid.hh"
#include "CustomNodes/ILLaserScan.hh"
#include "ILModel.cpp"
#include "ILUCS.cpp"

#include <ros/node.h>
#include <rosTF/rosTF.h>
#include <std_msgs/LaserScan.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include <pr2Core/pr2Core.h> /* Contains enumeration definitions for PR2 bodies and joints, must also include ros package pr2Core in manifest.xml */

class Vis3d
{
	public:
		///Camera views
		enum viewEnum{Maya,FPS,TFL,TFR,TRL,TRR,Top,Bottom,Front,Rear,Left,Right,MaxViewEnum};
		///Type of head laser scan display
		//enum scanType{Wipe, Replace, AtOnce, MaxScanType};
		enum scanType{Wipe, AtOnce, MaxScanType};
		///Marker insertion actions
		enum Action {newObject, modifyObject, deleteObject, MaxAction};
		///Marker insertion object types
		enum Type {Arrow, Cube, Sphere, Text, Custom, MaxType};
	//ros declarations
		ros::node *myNode;
		rosTFClient::rosTFClient tfClient;
		std_msgs::Empty shutHead;
		std_msgs::Empty shutFloor;
		std_msgs::Empty shutStereo;
		std_msgs::Empty transform;
		std_msgs::PointCloudFloat32 ptCldHead;
		std_msgs::LaserScan ptCldFloor;
		std_msgs::PointCloudFloat32 ptCldStereo;
		std_msgs::VisualizationMarker visMarker;
	//irrlicht declarations
		ILClient *localClient;
		ILRender *pLocalRenderer;
		//std::vector<ILPointCloud*> ilHeadCloud;
		ILLaserScan *ilHeadCloud;
		ILLaserScan *ilFloorCloud;
		ILLaserScan *ilStereoCloud;
		//std::vector<ILPointCloud*> ilStereoCloud;
		ILGrid *ilGrid;
		//std::vector<ILModel*> model;
		std::vector<ILModel*> markers;
		ILUCS *ilucs;
		irr::scene::ILightSceneNode *light[2];
		irr::scene::ICameraSceneNode *cameras[MaxViewEnum];
		irr::scene::ISceneNode *intermediate;
		irr::scene::IAnimatedMesh *ArrowMesh;

		///Vis3d constructor
		Vis3d(ros::node *aNode);
		///Vis3d destructor
		~Vis3d();
		///Vis3d disabler
		void disable();
		///Checks if the renderer is enabled
		bool isEnabled();
		///Switches to a new camera
		void changeView(int id);
		///Enables (draws) the head Hokuyo point cloud data
		void enableHead();
		///Enables (draws) the models of the robot
		void enableModel();
		///Enables (draws) the universal coordinate system
		void enableUCS();
		///Enables (draws) the grid which represents a flat floor
		void enableGrid();
		///Enables (draws) the lower Hokuyo's point cloud
		void enableFloor();
		///Enables (draws) the stereo vision's point cloud
		void enableStereo();
		///Enables all user inserted objects
		void enableObjects();
		///Disables the head Hokuyo laser data
		void disableHead();
		///Disables the lower Hokuyo's laser data
		void disableFloor();
		///Disables the stereo vision's data
		void disableStereo();
		///Disables the Universal Coordinate System markers
		void disableUCS();
		///Disables the floor grid
		void disableGrid();
		///Disables all robot models
		void disableModel();
		///Disables all user inserted objects (but does not delete them)
		void disableObjects();
		///Deletes all user inserted objects
		void deleteObjects();
		///(callback)Clears data in the head based Hokuyo point cloud
		void shutterHead();
		///(callback)Clears data in the lower Hokuyo point cloud
		void shutterFloor();
		///(callback)Clears data in the stereo vision's point cloud
		void shutterStereo();
		///(callback)Adds a point cloud to the head based Hokuyo's point cloud
		void addHeadCloud();
		///(callback)Adds a point cloud to the lower Hokuyo's point cloud
		void addFloorCloud();
		///(callback)Adds a point cloud to the stereo vision's point cloud
		void addStereoCloud();
		///Changes the method that the head cloud is displayed
		void changeHeadLaser(int choice);
		///(callback)Updates the model whenever a new transformation is posted
		void newTransform();
		///(callback)Creates, modifies, or deletes an object
		void newMarker();

	protected:
		///Type of scan (Wipe,Replace,AtOnce = 0,1,2)
		int scanT;
		///Scan direction.  Should be only -1 or 1.
		int scanDir;
		///Index of currently used head point cloud.
		int headVertScanCount;
		///Index of currently used stereo point cloud.
		int stereoVertScanCount;
		///Remembers if the user controlled objects are visible or hidded.
		bool objectsVisibility;

		std::map<std::string, ILModel*> m_modelMap;
		//std::map<std::string, std::string> m_nameMap;
};

#endif // VIS3D_H

