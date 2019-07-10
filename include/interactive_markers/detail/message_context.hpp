/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 * 
 * Author: David Gossow
 *//*
 * message_context.h
 *
 *  Created on: Jul 17, 2012
 *      Author: gossow
 */

#ifndef MESSAGE_CONTEXT_H_
#define MESSAGE_CONTEXT_H_

#include <tf/tf.h>

#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

namespace interactive_markers
{

template<class MsgT>
class MessageContext
{
public:
  MessageContext( tf::Transformer& tf,
      const std::string& target_frame,
      const typename MsgT::ConstPtr& msg,
      bool enable_autocomplete_transparency = true);

  MessageContext<MsgT>& operator=( const MessageContext<MsgT>& other );

  // transform all messages with timestamp into target frame
  void getTfTransforms();

  typename MsgT::Ptr msg;

  // return true if tf info is complete
  bool isReady();

private:

  void init();

  bool getTransform( std_msgs::Header& header, geometry_msgs::Pose& pose_msg );

  void getTfTransforms( std::vector<visualization_msgs::InteractiveMarker>& msg_vec, std::list<size_t>& indices );
  void getTfTransforms( std::vector<visualization_msgs::InteractiveMarkerPose>& msg_vec, std::list<size_t>& indices );

  // array indices of marker/pose updates with missing tf info
  std::list<size_t> open_marker_idx_;
  std::list<size_t> open_pose_idx_;
  tf::Transformer& tf_;
  std::string target_frame_;
  bool enable_autocomplete_transparency_;
};

class InitFailException: public tf::TransformException
{
public:
  InitFailException(const std::string errorDescription) : tf::TransformException(errorDescription) { ; }
};


}

#endif /* MESSAGE_CONTEXT_H_ */
