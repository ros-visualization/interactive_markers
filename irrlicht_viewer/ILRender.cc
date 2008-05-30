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

#include "ILRender.hh"

using namespace irr;

// Construction / Destruction
ILRender::ILRender(core::dimension2d<s32> resolution, video::E_DRIVER_TYPE driverType)
: m_enabled (false),
  m_initialized (false),
  m_pRenderThread (NULL)
{
  pthread_mutex_init(&m_renderMutex,0L);
  init(resolution,driverType); 
}

ILRender::~ILRender() {
  cleanup();
  pthread_mutex_destroy(&m_renderMutex);
}

bool ILRender::uninitialized() {
  return !m_initialized;
}

void ILRender::init(core::dimension2d<s32> resolution, video::E_DRIVER_TYPE driverTyped) {
  using namespace ros::thread::member_thread;
  lock();
  if(!m_enabled) {
    // Start render thread
    m_enabled = true;
    m_pRenderThread = startMemberFunctionThread(this, &ILRender::renderLoop);
  }
  unlock();
}

void ILRender::cleanup() {
  lock();
  if(m_enabled && m_pRenderThread != NULL) {
    m_enabled = false;
    pthread_join(*m_pRenderThread, 0L);
    m_pRenderThread = NULL;
    m_pDevice->drop();
  }
  unlock();
}

// Renderer Management
void ILRender::lock() {
  if(pthread_mutex_lock(&m_renderMutex) == -1) {
    //TODO: throw exception here
  }
}

void ILRender::unlock() {
  if(pthread_mutex_unlock(&m_renderMutex) == -1) {
    //TODO: throw exception here
  }
}

// Node Management
void ILRender::addNode(scene::ISceneNode *pNode) {
  lock();

  // Check for node first, to ensure only one copy
  if(m_nodes.find(pNode) == m_nodes.end()) {
    // Initialize nodeInfo structure
    NodeInfo *tempinfo = new NodeInfo();

    // Lock mutex before insertion
    int ret = pthread_mutex_lock(&(tempinfo->lock));
    if(ret != 0) {
      //TODO: throw exception here
    }
    // Insert
    m_nodes.insert(std::make_pair(pNode,tempinfo));
    // Unlock mutex
    unlock(pNode);
  }

  unlock();
}

void ILRender::remNode(scene::ISceneNode *pNode) {
  lock();

  lock(pNode);
  delete m_nodes[pNode];
  m_nodes.erase(pNode);
  
  unlock();
}

void ILRender::pushToDraw(scene::ISceneNode *pNode) {
  m_drawPipe.push(pNode);
}

void ILRender::enable(scene::ISceneNode *pNode) {
  lock(pNode);
  m_nodes[pNode]->enabled = true;
  unlock(pNode);
}

void ILRender::disable(scene::ISceneNode *pNode) {
  lock(pNode);
  m_nodes[pNode]->enabled = false;
  unlock(pNode);
}

// Functions for locking individual nodes
bool ILRender::trylock(scene::ISceneNode *pNode) {
  int ret = pthread_mutex_trylock(&(m_nodes[pNode]->lock));
  if(ret != 0 && errno == EBUSY) {
    return false;
  }
  return true;
}

void ILRender::lock(scene::ISceneNode *pNode) {
  int ret = pthread_mutex_lock(&(m_nodes[pNode]->lock));
  if(ret != 0) {
    //TODO: throw exception here
  }
}

void ILRender::unlock(scene::ISceneNode *pNode) {
  int ret = pthread_mutex_unlock(&(m_nodes[pNode]->lock));
  if(ret != 0) {
    //TODO: throw exception here
  }
}

// Render Loop
void ILRender::renderLoop() {

  std::cerr<<"RENDERLOOP!"<<std::endl;

  lock();
  m_pDevice = createDevice(video::EDT_OPENGL, core::dimension2d<s32>(640, 480));

  if (m_pDevice == 0) {
    // TODO: REPLACE THIS WITH AN EXCEPTION
    exit(-1);
  }

  m_pDriver = m_pDevice->getVideoDriver();
  m_pManager = m_pDevice->getSceneManager();
#if 1
  m_pManager->addCameraSceneNodeMaya(NULL,-1500.0f,200.0f,1500.0f);
#else
  SKeyMap keyMap[8];
  keyMap[0].Action = EKA_MOVE_FORWARD;
  keyMap[0].KeyCode = KEY_UP;
  keyMap[1].Action = EKA_MOVE_FORWARD;
  keyMap[1].KeyCode = KEY_KEY_W;

  keyMap[2].Action = EKA_MOVE_BACKWARD;
  keyMap[2].KeyCode = KEY_DOWN;
  keyMap[3].Action = EKA_MOVE_BACKWARD;
  keyMap[3].KeyCode = KEY_KEY_S;

  keyMap[4].Action = EKA_STRAFE_LEFT;
  keyMap[4].KeyCode = KEY_LEFT;
  keyMap[5].Action = EKA_STRAFE_LEFT;
  keyMap[5].KeyCode = KEY_KEY_A;

  keyMap[6].Action = EKA_STRAFE_RIGHT;
  keyMap[6].KeyCode = KEY_RIGHT;
  keyMap[7].Action = EKA_STRAFE_RIGHT;
  keyMap[7].KeyCode = KEY_KEY_D;

  m_pManager->addCameraSceneNodeFPS(0, 1000, 100, -1, keyMap, 8);
#endif
  m_pDevice->getCursorControl()->setVisible(true);

  m_initialized = true;
  unlock();

  std::cerr<<"ILRENDER INITIALIZED!"<<std::endl;

  while(m_enabled) {
    if(m_pDevice->run()) {
      lock();
      
      // Draw Scene
      m_pDriver->beginScene(true, true, video::SColor(255,0,0,0));
      m_pManager->drawAll();
      m_pDriver->endScene();

      unlock();
    } else {
      m_enabled = false;
    }
    usleep(10000);
  }

  pthread_exit(NULL);
}

// Accessors
IrrlichtDevice* ILRender::device() {
  return m_pDevice;
}

video::IVideoDriver* ILRender::driver() {
  return m_pDriver;
}

scene::ISceneManager* ILRender::manager() {
  return m_pManager;
}
