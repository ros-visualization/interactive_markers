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

#include <iostream>
#include <math.h>

#include <irrlicht.h>

#include "ILRender.hh"
#include "ILClient.hh"
#include "CustomNodes/ILPointCloud.hh"

// Global enabled flag
bool g_enabled = true;

// Function to test async node manipulation
void* asyncClient(void *args) {
  using namespace irr;
  
  // Load args
  int index = ((int*)args)[0];
  double direction = ((double*)args)[1];
  double radius = ((double*)args)[2];
  double speed = ((double*)args)[3];

  // Create client, and grab a local pointer to the renderer and lock it
  ILClient localClient;
  ILRender *pLocalRenderer = ILClient::getSingleton();

  // Create node (a sphere)
  // NOTE: When accessing these functions we need to surround the calls with a
  // lock/unlock pair
  pLocalRenderer->lock();  
  scene::ISceneNode *sphere = pLocalRenderer->manager()->addSphereSceneNode(1, 100, pLocalRenderer->manager()->getRootSceneNode(), index);
  //ILPointCloud *sphere = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 666);
  pLocalRenderer->unlock();  

  // Register scene node
  // NOTE: these functions are atomic and provide their own thread protection
  pLocalRenderer->addNode(sphere);
  pLocalRenderer->enable(sphere);

  // Working variables in loop
  core::vector3df pos;
  double angle = 0;

  // Continuously manipulate node
  while(g_enabled) {
    // Acquire lock on the node
    pLocalRenderer->lock(sphere);

    // Move it a little around a circle
    angle += speed;
    if(angle >= 2*M_PI) { angle = 0; }
    pos = core::vector3df((direction)*(radius)*(-1)*sin(angle),0,(direction)*(radius)*cos(angle));
    sphere->setPosition(pos);

    // Relinquish the lock on the node
    pLocalRenderer->unlock(sphere);

    // Wait a little
    usleep(10000);
  }
  
  // Cleanup resources
  pLocalRenderer->remNode(sphere);

  pthread_exit(NULL);
}

int main(int argc, char** argv) {

  // Spawn 3 clients
  pthread_t pClient1, pClient2, pClient3;
  double rgClientArg1[4] = {1,1,5,0.01};
  double rgClientArg2[4] = {2,-1,10,0.005};
  double rgClientArg3[4] = {3,1,15,0.1};

  pthread_create(&pClient1, 0L, asyncClient, rgClientArg1);
  pthread_create(&pClient2, 0L, asyncClient, rgClientArg2);
  pthread_create(&pClient3, 0L, asyncClient, rgClientArg3);

  // Wait 15 seconds, then disable rendering
  sleep(20);
  g_enabled = false;

  // Join threads
  pthread_join(pClient1,0L);
  pthread_join(pClient2,0L);
  pthread_join(pClient3,0L);

  return 0;
}
