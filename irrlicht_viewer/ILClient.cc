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

#include "ILClient.hh"

// Initialize singletona members
ILRender *ILClient::s_pRenderer = NULL;
uint32_t ILClient::s_numClients = 0;
pthread_mutex_t *ILClient::s_pSingletonMutex = NULL;

ILClient::ILClient() {
  // If it is uninialized, initialize mutex for singleton access
  if(s_pSingletonMutex == NULL) {
    s_pSingletonMutex = new pthread_mutex_t;
    pthread_mutex_init(s_pSingletonMutex,0L);
  }

  pthread_mutex_lock(s_pSingletonMutex);

  // Increment client counter
  s_numClients++;

  pthread_mutex_unlock(s_pSingletonMutex);
}

ILClient::~ILClient() {
  // Acquire singleton lock
  pthread_mutex_lock(s_pSingletonMutex);

  // Decrement client counter
  s_numClients--;

  // If this is the last client
  if(s_numClients == 0) {
    // Destruct the singleton members
    delete s_pRenderer;
    s_pRenderer = NULL;
    pthread_mutex_destroy(s_pSingletonMutex);
    delete s_pSingletonMutex;
  } 
  else {
    // Release the lock
    pthread_mutex_unlock(s_pSingletonMutex);
  }
}

ILRender* ILClient::getSingleton() {
  pthread_mutex_lock(s_pSingletonMutex);

  // Check for renderer instantiation
  if(s_pRenderer == NULL) {
    // Create a default ILRender
    s_pRenderer = new ILRender();

    while(s_pRenderer->uninitialized()) {
      usleep(10);
    }
  }

  pthread_mutex_unlock(s_pSingletonMutex);

  return s_pRenderer;
}
