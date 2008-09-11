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

class LogPage;
class NodePage;

typedef std::vector<LogPage*> V_LogPage;
typedef std::vector<NodePage*> V_NodePage;

/**
 * \class RosoutPanel
 * \brief An embeddable panel which listens on rosout and displays any messages that arrive.
 */
class RosoutPanel : public RosoutPanelBase
{
public:
  /**
   * \brief Constructor
   * @param parent The window which is the parent of this one
   */
  RosoutPanel( wxWindow* parent );
  ~RosoutPanel();

  /**
   * \brief Set this panel to be enabled or not.
   *
   * When enabled, it will be subscribed to the rosout topic and processing messages.  When disabled, it will not.
   * @param enabled Should we be enabled?
   */
	void setEnabled(bool enabled);
	/**
	 * \brief Set the topic to listen on for rostools::Log messages
	 * @param topic The topic name
	 */
  void setTopic( const std::string& topic );

  /**
   * \brief Clear all messages
   */
  void clear();

  /**
   * \brief Mostly for internal use.  Creates a page in our choicebook for a node.
   * @param name The name of the page.  If a page of this name already exists, it will return the pre-existing one.
   * @return If no page of the same name existed, a new page.  Otherwise, the pre-existing page.
   */
	NodePage* createNodePage( const std::string& name );
	/**
	 * \brief Mostly for internal use.  Creates the default log pages (All, Fatal, Error, Warning, Debug, Info)
	 * @param node_page The node page to create the log pages under
	 */
	void createDefaultLogPages( NodePage* node_page );
	/**
	 * \brief Mostly for internal use.  Creates a log page.
	 * @param node_page The node page to create the log page under
	 * @param filter A flags-word for which types of messages will appear in this log page.  Flags should be ored together from the flags available in rostools::Log
	 * @param name Name of this log page
	 * @return The new log page
	 */
	LogPage* createLogPage( NodePage* node_page, uint32_t filter, const std::string& name );

protected:
  /**
   * \brief (wx callback) Called when the "Enable" checkbox is toggled
   */
  virtual void onEnable( wxCommandEvent& event );
  /**
   * \brief (wx callback) Called when the "Setup" button is pressed
   */
  virtual void onSetup( wxCommandEvent& event );
  /**
   * \brief (wx callback) Called when the "Pause" button is toggled
   */
  virtual void onPauseToggled( wxCommandEvent& event );
  /**
   * \brief (wx callback) Called when the "Clear" button is pressed
   */
  virtual void onClear( wxCommandEvent& event );
  /**
   * \brief (wx callback) Called every 100ms so we can process new messages
   */
  void onProcessTimer( wxTimerEvent& evt );

  /**
   * \brief subscribe to our topic
   */
  void subscribe();
  /**
   * \brief unsubscribe from our topic
   */
  void unsubscribe();

  /**
   * \brief Calls a callback for every log page
   * @param f The callback
   */
  void forEachLogPage( boost::function<void (NodePage* node_page, LogPage* log_page)> f );

  /**
   * \brief Creates the default node and log pages
   */
  void createDefaultPages();
  /**
   * \brief (ros callback) Called when there is a new message waiting
   */
  void incomingMessage();
  /**
   * \brief Processes any messages in our message queue
   */
  void processMessages();
  /**
   * \brief Process a log message
   * @param message The message to process
   */
  void processMessage( const rostools::Log& message );

  bool enabled_;                                            ///< Are we enabled?
  std::string topic_;                                       ///< The topic we're listening on (or will listen on once we're enabled)

  ros::node* ros_node_;                                     ///< Our pointer to the global ros::node
  rostools::Log message_;                                   ///< Our incoming message

  typedef std::vector<rostools::Log> V_Log;
  V_Log message_queue_;                                     ///< Queue of messages we've received since the last time processMessages() was called
  ros::thread::mutex queue_mutex_;                          ///< Mutex for locking the message queue

  V_NodePage node_pages_;                                   ///< List of node pages in the order they were added
  typedef std::map<std::string, NodePage*> M_NameToNodePage;
  M_NameToNodePage node_pages_by_name_;                     ///< Map of name to node page

  wxTimer* process_timer_;                                  ///< Timer used to periodically process messages
};

#endif // WX_ROSOUT_ROSOUT_PANEL
