/*
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
      const typename MsgT::ConstPtr& msg);

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
};

class InitFailException: public tf::TransformException
{
public:
  InitFailException(const std::string errorDescription) : tf::TransformException(errorDescription) { ; };
};


}

#endif /* MESSAGE_CONTEXT_H_ */
