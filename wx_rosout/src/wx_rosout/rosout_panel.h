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

/*
 * wx panel for viewing rosout.
 *
 * Written by Josh Faust
 */
#ifndef WX_ROSOUT_ROSOUT_PANEL
#define WX_ROSOUT_ROSOUT_PANEL

/**
@mainpage

@htmlinclude manifest.html

*/

#include "rosout_generated.h"
#include "rostools/Log.h"

#include <string>
#include <vector>
#include <map>
#include <rosthread/mutex.h>

#include <boost/function.hpp>

#include <wx/font.h>

namespace ros
{
class node;
}

class wxTimer;
class wxTimerEvent;
class wxAuiNotebook;
class wxRichTextCtrl;

struct LogPage
{
  uint32_t filter_;
  wxTextCtrl* text_control_;
};
typedef std::vector<LogPage*> V_LogPage;

struct NodePage
{
  NodePage() {}
  ~NodePage();
  std::string name_;
  wxAuiNotebook* notebook_;

  V_LogPage log_pages_;
};
typedef std::vector<NodePage*> V_NodePage;

class RosoutPanel : public RosoutPanelBase
{
public:
  RosoutPanel( wxWindow* parent );
  ~RosoutPanel();

	void setEnabled(bool enabled);
  void setTopic( const std::string& topic );

  void clear();

	NodePage* createNodePage( const std::string& name );
	void createDefaultLogPages( NodePage* node_page );
	LogPage* createLogPage( NodePage* node_page, uint32_t filter, const std::string& name );

protected:
  virtual void onEnable( wxCommandEvent& event );
  virtual void onSetup( wxCommandEvent& event );
  virtual void onPauseToggled( wxCommandEvent& event );
  virtual void onClear( wxCommandEvent& event );
  void onProcessTimer( wxTimerEvent& evt );

  void subscribe();
  void unsubscribe();

  void forEachLogPage( boost::function<void (NodePage* node_page, LogPage* log_page)> f );

  void createDefaultPages();
  void incomingMessage();
  void processMessages();
  void processMessage( const rostools::Log& message );

  bool enabled_;
  std::string topic_;

  ros::node* ros_node_;
  rostools::Log message_;

  typedef std::vector<rostools::Log> V_Log;
  V_Log message_queue_;
  ros::thread::mutex queue_mutex_;

  V_NodePage node_pages_;
  typedef std::map<std::string, NodePage*> M_NameToNodePage;
  M_NameToNodePage node_pages_by_name_;

  wxTimer* process_timer_;
};

#endif // WX_ROSOUT_ROSOUT_PANEL
