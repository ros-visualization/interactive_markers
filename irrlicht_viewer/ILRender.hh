#ifndef __IL_RENDER_HH
#define __IL_RENDER_HH

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


// System Includes
#include <stdint.h>
#include <pthread.h>
#include <errno.h>

#include <iostream>
#include <queue>
#include <map>

// IrrLicht
#include <irrlicht.h>

#include <rosthread/member_thread.h>

class ILRender {
public:
  // Constructors
  ILRender(irr::core::dimension2d<irr::s32> resolution = irr::core::dimension2d<irr::s32>(640, 480),
           irr::video::E_DRIVER_TYPE driverType = irr::video::EDT_OPENGL);
  ~ILRender();

  // Scene Management
  // ILRender::init creates the underlying IrrLicht renderer, and spawns a thread
  // executing ILRender::renderLoop
  void init(irr::core::dimension2d<irr::s32> resolution,irr::video::E_DRIVER_TYPE driverType);
  bool uninitialized();

  // ILRender::cleanup halts rendering and destroys the underlying IrrLicht
  // renderer. 
  void cleanup();

  // Renderer Management
  void lock();
  void unlock();

  // Node Management
  void addNode(irr::scene::ISceneNode *pNode);
  void remNode(irr::scene::ISceneNode *pNode);

  // If you do not wish to remove a node, but do want to disable/hide it, you
  // can use these methods. They simply manipulate a boolean value associated
  // with each registered node.
  void enable(irr::scene::ISceneNode *pNode);
  void disable(irr::scene::ISceneNode *pNode);

  void pushToDraw(irr::scene::ISceneNode *pNode);

  // If a client in another thread wishes to modify one of the scene nodes,
  // they must lock the specific node by calling the following methods. These
  // each manipulate a pthread_mutex_t which is associated with the given node
  //
  // When rendering, the ILRender class calls trylock, attempting to get a lock
  // on the node. If it is being maniplated by another thread at that point,
  // pthread_mutex_trylock will report EBUSY and ILRender will not try to draw
  // the node.
  void lock(irr::scene::ISceneNode *pNode);
  bool trylock(irr::scene::ISceneNode *pNode);
  void unlock(irr::scene::ISceneNode *pNode);
  
  // Accessors
  irr::IrrlichtDevice* device();
  irr::video::IVideoDriver* driver();
  irr::scene::ISceneManager* manager();

  bool isEnabled(){return m_enabled;};

private:
  // Flags
  bool m_enabled;
  bool m_initialized;
  
  // IrrLicht objects
  irr::IrrlichtDevice *m_pDevice;
  irr::video::IVideoDriver *m_pDriver;
  irr::scene::ISceneManager *m_pManager;

  // Loop for drawing
  // This function is executed in its own thread when ILRender::init is called.
  // It iterates through the list of registered nodes, drawing each of them if
  // the node meets both conditions:
  //  1. the node is enabled
  //  2. the ILRender can acquire the lock in the node's nodeInfo structure
  void renderLoop();

  // Thread objects
  pthread_t *m_pRenderThread;
  pthread_mutex_t m_renderMutex;

  std::queue<irr::scene::ISceneNode*> m_drawPipe;

  // Map of nodes in this renderer and their respective draw states
  // Each node pointer is associated with a nodeInfo struct
  //  This struct holds an enabled flag as well as a mutex for multithreaded
  //  access. It's constructor and destructor properly handle the
  //  initialization and the destruction of the mutex.
  // If a client in another thread wishes to modify one of the scene nodes,
  // they must lock the specific node by calling ILRender::lock,unlock
  
  class NodeInfo {
  public:
    bool enabled;
    bool needs_update;
    pthread_mutex_t lock;
    NodeInfo() {
      enabled = false;
      needs_update = false;
      if(pthread_mutex_init(&lock,0L) != 0) { throw -1; }
    }
    ~NodeInfo() {
      pthread_mutex_destroy(&lock);
    }
  };

  std::map<irr::scene::ISceneNode*,NodeInfo*> m_nodes;
  std::map<irr::scene::ISceneNode*,NodeInfo*>::iterator m_nodeIter;
};

#endif // ifndef __IL_RENDER_HH
