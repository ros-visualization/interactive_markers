/*
 * Copyright (c) 2011, Willow Garage, Inc.
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


#include <ros/ros.h>
#include <ros/console.h>

#include <gtest/gtest.h>

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/interactive_marker_client.h>

#define DBG_MSG( ... ) printf( __VA_ARGS__ ); printf("\n");
#define DBG_MSG_STREAM( ... )  std::cout << __VA_ARGS__ << std::endl;

using namespace interactive_markers;

struct Msg
{
  enum {
    INIT,
    KEEP_ALIVE,
    UPDATE,
    POSE,
    DELETE,
    TF_INFO
  } type;

  Msg()
  {
    type = INIT;
    seq_num = 0;
  }

  uint64_t seq_num;

  std::string server_id;
  std::string frame_id;
  ros::Time stamp;

  std::vector<std::string> expect_reset_calls;
  std::vector<int> expect_init_seq_num;
  std::vector<int> expect_update_seq_num;
};

std::string target_frame = "target_frame";


class SequenceTest
{
  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;

  std::vector<visualization_msgs::InteractiveMarkerInit> recv_init_msgs;
  std::vector<visualization_msgs::InteractiveMarkerUpdate> recv_update_msgs;
  std::vector<std::string> recv_reset_calls;

  void resetReceivedMsgs()
  {
    recv_init_msgs.clear();
    recv_update_msgs.clear();
    recv_reset_calls.clear();
  }

  void updateCb( const UpdateConstPtr& msg )
  {
    DBG_MSG_STREAM( "updateCb called." );
    recv_update_msgs.push_back( *msg );
  }

  void initCb( const InitConstPtr& msg )
  {
    DBG_MSG_STREAM( "initCb called." );
    recv_init_msgs.push_back( *msg );
  }

  void statusCb( InteractiveMarkerClient::StatusT status,
      const std::string& server_id,
      const std::string& msg )
  {
    std::string status_string[]={"INFO","WARN","ERROR"};
    ASSERT_TRUE( (unsigned)status < 3 );
    DBG_MSG_STREAM( "(" << status_string[(unsigned)status] << ") " << server_id << ": " << msg );
  }

  void resetCb( const std::string& server_id )
  {
    DBG_MSG_STREAM( "resetCb called." );
    recv_reset_calls.push_back(server_id);
  }

public:
  void test( std::vector<Msg> messages )
  {
    tf::Transformer tf;

    //tf.setTransform();

    // create an interactive marker server on the topic namespace simple_marker

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.pose.orientation.w=1;

    std::string topic_ns ="im_client_test";

    interactive_markers::InteractiveMarkerClient client(tf, target_frame, topic_ns );

    client.setInitCb( boost::bind(&SequenceTest::initCb, this, _1 ) );
    client.setUpdateCb( boost::bind(&SequenceTest::updateCb, this, _1 ) );
    client.setResetCb( boost::bind(&SequenceTest::resetCb, this, _1 ) );
    client.setStatusCb( boost::bind(&SequenceTest::statusCb, this, _1, _2, _3 ) );

    std::map< int, visualization_msgs::InteractiveMarkerInit > sent_init_msgs;
    std::map< int, visualization_msgs::InteractiveMarkerUpdate > sent_update_msgs;

    for ( size_t i=0; i<messages.size(); i++ )
    {
      resetReceivedMsgs();

      Msg& msg = messages[i];

      int_marker.header.frame_id=msg.frame_id;
      int_marker.header.stamp=msg.stamp;
      int_marker.pose.orientation.w = 1.0;

      std::ostringstream s;
      s << i;
      int_marker.name=s.str();

      switch( msg.type )
      {
      case Msg::INIT:
      {
        DBG_MSG_STREAM( i << " INIT: seq_num=" << msg.seq_num << " frame=" << msg.frame_id << " stamp=" << msg.stamp );
        visualization_msgs::InteractiveMarkerInitPtr init_msg_out( new visualization_msgs::InteractiveMarkerInit() );
        init_msg_out->server_id=msg.server_id;
        init_msg_out->seq_num=msg.seq_num;
        init_msg_out->markers.push_back( int_marker );
        client.processInit( init_msg_out );
        sent_init_msgs[msg.seq_num]=*init_msg_out;
        break;
      }
      case Msg::KEEP_ALIVE:
      {
        DBG_MSG_STREAM( i << " KEEP_ALIVE: seq_num=" << msg.seq_num );
        visualization_msgs::InteractiveMarkerUpdatePtr update_msg_out( new visualization_msgs::InteractiveMarkerUpdate() );
        update_msg_out->server_id=msg.server_id;
        update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE;
        update_msg_out->seq_num=msg.seq_num;

        client.processUpdate( update_msg_out );
        sent_update_msgs[msg.seq_num]=*update_msg_out;
        break;
      }
      case Msg::UPDATE:
      {
        DBG_MSG_STREAM( i << " UPDATE: seq_num=" << msg.seq_num << " frame=" << msg.frame_id << " stamp=" << msg.stamp );
        visualization_msgs::InteractiveMarkerUpdatePtr update_msg_out( new visualization_msgs::InteractiveMarkerUpdate() );
        update_msg_out->server_id=msg.server_id;
        update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
        update_msg_out->seq_num=msg.seq_num;

        update_msg_out->markers.push_back( int_marker );
        client.processUpdate( update_msg_out );
        sent_update_msgs[msg.seq_num]=*update_msg_out;
        break;
      }
      case Msg::POSE:
      {
        DBG_MSG_STREAM( i << " POSE: seq_num=" << msg.seq_num << " frame=" << msg.frame_id << " stamp=" << msg.stamp );
        visualization_msgs::InteractiveMarkerUpdatePtr update_msg_out( new visualization_msgs::InteractiveMarkerUpdate() );
        update_msg_out->server_id=msg.server_id;
        update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
        update_msg_out->seq_num=msg.seq_num;

        visualization_msgs::InteractiveMarkerPose pose;
        pose.header=int_marker.header;
        pose.name=int_marker.name;
        pose.pose=int_marker.pose;
        update_msg_out->poses.push_back( pose );
        client.processUpdate( update_msg_out );
        sent_update_msgs[msg.seq_num]=*update_msg_out;
        break;
      }
      case Msg::DELETE:
      {
        DBG_MSG_STREAM( i << " DELETE: seq_num=" << msg.seq_num );
        visualization_msgs::InteractiveMarkerUpdatePtr update_msg_out( new visualization_msgs::InteractiveMarkerUpdate() );
        update_msg_out->server_id="/im_client_test/test_server";
        update_msg_out->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
        update_msg_out->seq_num=msg.seq_num;

        update_msg_out->erases.push_back( int_marker.name );
        client.processUpdate( update_msg_out );
        sent_update_msgs[msg.seq_num]=*update_msg_out;
        break;
      }
      case Msg::TF_INFO:
      {
        DBG_MSG_STREAM( i << " TF_INFO: " << msg.frame_id << " -> " << target_frame << " at time " << msg.stamp.toSec() );
        tf::StampedTransform stf;
        stf.frame_id_=msg.frame_id;
        stf.child_frame_id_=target_frame;
        stf.stamp_=msg.stamp;
        stf.setIdentity();
        tf.setTransform( stf, msg.server_id );
        break;
      }
      }

      /*
      ASSERT_EQ( 0, recv_update_msgs.size()  );
      ASSERT_EQ( 0, recv_init_msgs.size()  );
      ASSERT_EQ( 0, recv_reset_calls.size()  );
      */

      client.update();

      ASSERT_EQ( msg.expect_update_seq_num.size(), recv_update_msgs.size()  );
      ASSERT_EQ( msg.expect_init_seq_num.size(), recv_init_msgs.size()  );
      ASSERT_EQ( msg.expect_reset_calls.size(), recv_reset_calls.size()  );

      for ( size_t u=0; u<msg.expect_update_seq_num.size(); u++ )
      {
        ASSERT_TRUE( sent_update_msgs.find(msg.expect_update_seq_num[u]) != sent_update_msgs.end() );

        visualization_msgs::InteractiveMarkerUpdate sent_msg = sent_update_msgs[msg.expect_update_seq_num[u]];
        visualization_msgs::InteractiveMarkerUpdate recv_msg = recv_update_msgs[ u ];

        //sanity check
        ASSERT_EQ( sent_msg.seq_num, msg.expect_update_seq_num[u]  );

        //chech sequence number
        ASSERT_EQ( recv_msg.seq_num, msg.expect_update_seq_num[u]  );

        // check sent/received messages for equality
        ASSERT_EQ( recv_msg.markers.size(), sent_msg.markers.size()  );
        ASSERT_EQ( recv_msg.poses.size(), sent_msg.poses.size()  );
        ASSERT_EQ( recv_msg.erases.size(), sent_msg.erases.size()  );

        // check that messages are equal
        // and everything is transformed into the target frame
        for ( size_t m=0; m<sent_msg.markers.size(); m++ )
        {
          ASSERT_EQ( recv_msg.markers[m].name, sent_msg.markers[m].name  );
          ASSERT_EQ( recv_msg.markers[m].header.stamp, sent_msg.markers[m].header.stamp  );
          if ( sent_msg.markers[m].header.stamp == ros::Time(0) )
          {
            ASSERT_EQ( target_frame, sent_msg.markers[m].header.frame_id  );
          }
          else
          {
            ASSERT_EQ( target_frame, recv_msg.markers[m].header.frame_id  );
          }
        }
        for ( size_t p=0; p<sent_msg.poses.size(); p++ )
        {
          ASSERT_EQ( recv_msg.poses[p].name, sent_msg.poses[p].name  );
          ASSERT_EQ( recv_msg.poses[p].header.stamp, sent_msg.poses[p].header.stamp  );
          if ( sent_msg.poses[p].header.stamp == ros::Time(0) )
          {
            ASSERT_EQ( target_frame, sent_msg.poses[p].header.frame_id  );
          }
          else
          {
            ASSERT_EQ( target_frame, recv_msg.poses[p].header.frame_id  );
          }
        }
        for ( size_t e=0; e<sent_msg.erases.size(); e++ )
        {
          ASSERT_EQ( recv_msg.erases[e], sent_msg.erases[e]  );
        }
      }


      for ( size_t u=0; u<msg.expect_init_seq_num.size(); u++ )
      {
        ASSERT_TRUE( sent_init_msgs.find(msg.expect_init_seq_num[u]) != sent_init_msgs.end() );

        visualization_msgs::InteractiveMarkerInit sent_msg = sent_init_msgs[msg.expect_init_seq_num[u]];
        visualization_msgs::InteractiveMarkerInit recv_msg = recv_init_msgs[ u ];

        //sanity check
        ASSERT_EQ( sent_msg.seq_num, msg.expect_init_seq_num[u]  );

        //chech sequence number
        ASSERT_EQ( recv_msg.seq_num, msg.expect_init_seq_num[u]  );

        // check sent/received messages for equality
        ASSERT_EQ( recv_msg.markers.size(), sent_msg.markers.size()  );

        // check that messages are equal
        // and everything is transformed into the target frame
        for ( size_t m=0; m<sent_msg.markers.size(); m++ )
        {
          ASSERT_EQ( recv_msg.markers[m].name, sent_msg.markers[m].name  );
          ASSERT_EQ( recv_msg.markers[m].header.stamp, sent_msg.markers[m].header.stamp  );
          if ( sent_msg.markers[m].header.stamp == ros::Time(0) )
          {
            // check if we can transform frame-locked messages
            ASSERT_TRUE( tf.canTransform( target_frame, sent_msg.markers[m].header.frame_id, ros::Time(0) ) );
          }
          else
          {
            // check if non-framelocked messages are being transformed for us
            ASSERT_EQ( target_frame, recv_msg.markers[m].header.frame_id  );
          }
        }
      }
    }
  }
};

TEST(InteractiveMarkerClient, init_simple1)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, init_simple2)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


TEST(InteractiveMarkerClient, init_many_inits)
{
  Msg msg;

  std::vector<Msg> seq;

  for ( int i=0; i<200; i++ )
  {
    msg.type=Msg::INIT;
    msg.seq_num=i;
    msg.server_id="server1";
    msg.frame_id=target_frame;
    seq.push_back(msg);
  }

  // this update should be ommitted
  msg.type=Msg::UPDATE;
  msg.expect_init_seq_num.push_back(msg.seq_num);
  seq.push_back(msg);

  // this update should go through
  msg.seq_num++;
  msg.expect_init_seq_num.clear();
  msg.expect_update_seq_num.push_back(msg.seq_num);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, init_many_updates)
{
  Msg msg;

  std::vector<Msg> seq;

  for ( int i=0; i<200; i++ )
  {
    msg.type=Msg::UPDATE;
    msg.seq_num=i;
    msg.server_id="server1";
    msg.frame_id=target_frame;
    seq.push_back(msg);
  }

  msg.type=Msg::INIT;
  msg.seq_num=190;
  msg.expect_init_seq_num.push_back(msg.seq_num);
  for ( int i=191; i<200; i++ )
  {
    msg.expect_update_seq_num.push_back(i);
  }
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, init_invalid_tf)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server1";
  msg.frame_id="invalid_frame";
  seq.push_back(msg);

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, init_wait_tf)
{
  Msg msg;

  std::vector<Msg> seq;

  // initial tf info needed so wait_frame is in the tf tree
  msg.type=Msg::TF_INFO;
  msg.server_id="server1";
  msg.frame_id="wait_frame";
  msg.stamp=ros::Time(1.0);
  seq.push_back(msg);

  // send init message that lives in the future
  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.stamp=ros::Time(2.0);
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  seq.push_back(msg);

  // send tf info -> message should get passed through
  msg.type=Msg::TF_INFO;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  // send update message that lives in the future
  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.stamp=ros::Time(3.0);
  msg.expect_init_seq_num.clear();
  seq.push_back(msg);

  // send tf info -> message should get passed through
  msg.type=Msg::TF_INFO;
  msg.expect_update_seq_num.push_back(1);
  seq.push_back(msg);

  // send pose message that lives in the future
  msg.type=Msg::POSE;
  msg.seq_num=2;
  msg.stamp=ros::Time(4.0);
  msg.expect_update_seq_num.clear();
  seq.push_back(msg);

  // send tf info -> message should get passed through
  msg.type=Msg::TF_INFO;
  msg.expect_update_seq_num.push_back(2);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


TEST(InteractiveMarkerClient, init_wait_tf_zerotime)
{
  Msg msg;

  std::vector<Msg> seq;

  // send init message with zero timestamp and non-existing tf frame
  msg.type=Msg::INIT;
  msg.server_id="server1";
  msg.frame_id="wait_frame";
  msg.seq_num=0;
  msg.stamp=ros::Time(0.0);
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  seq.push_back(msg);

  // send tf info -> message should get passed through
  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(1.0);
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


TEST(InteractiveMarkerClient, init_wait_tf_inverse)
{
  // send messages with timestamps going backwards
  // they should still be delivered in the right order
  // according to their seq_num

  Msg msg;

  std::vector<Msg> seq;

  // initial tf info needed so wait_frame is in the tf tree
  msg.type=Msg::TF_INFO;
  msg.server_id="server1";
  msg.frame_id="wait_frame";
  msg.stamp=ros::Time(1.0);
  seq.push_back(msg);

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.stamp=ros::Time(6);
  seq.push_back(msg);

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.stamp=ros::Time(5);
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.seq_num=0;
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.stamp=ros::Time(5);
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.stamp=ros::Time(4);
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.seq_num=3;
  msg.stamp=ros::Time(3);
  seq.push_back(msg);

  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(2);
  seq.push_back(msg);

  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(3);
  seq.push_back(msg);

  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(4);
  seq.push_back(msg);

  // as soon as tf info for init #1 is there,
  // all messages should go through
  msg.stamp=ros::Time(5);
  msg.expect_init_seq_num.push_back(1);
  msg.expect_update_seq_num.push_back(2);
  msg.expect_update_seq_num.push_back(3);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wait_tf_inverse)
{
  // send messages with timestamps going backwards
  // they should still be delivered in the right order
  // according to their seq_num

  Msg msg;

  std::vector<Msg> seq;

  // initial tf info needed so wait_frame is in the tf tree
  msg.type=Msg::TF_INFO;
  msg.server_id="server1";
  msg.frame_id="wait_frame";
  msg.stamp=ros::Time(1);
  seq.push_back(msg);

  msg.type=Msg::INIT;
  msg.seq_num=0;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.seq_num=0;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  // init complete

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.stamp=ros::Time(5);
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.stamp=ros::Time(4);
  seq.push_back(msg);

  msg.type=Msg::UPDATE;
  msg.seq_num=3;
  msg.stamp=ros::Time(3);
  seq.push_back(msg);

  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(3);
  seq.push_back(msg);

  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(4);
  seq.push_back(msg);

  // all messages should go through in the right order
  msg.type=Msg::TF_INFO;
  msg.stamp=ros::Time(5);
  msg.expect_update_seq_num.push_back(1);
  msg.expect_update_seq_num.push_back(2);
  msg.expect_update_seq_num.push_back(3);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


TEST(InteractiveMarkerClient, wrong_seq_num1)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::KEEP_ALIVE;
  msg.seq_num=1;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wrong_seq_num2)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::KEEP_ALIVE;
  msg.seq_num=0;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wrong_seq_num3)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wrong_seq_num4)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.expect_update_seq_num.push_back(2);
  seq.push_back(msg);

  msg.expect_update_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wrong_seq_num5)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.expect_update_seq_num.push_back(2);
  seq.push_back(msg);

  msg.expect_update_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}

TEST(InteractiveMarkerClient, wrong_seq_num6)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=1;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=2;
  msg.expect_update_seq_num.push_back(2);
  seq.push_back(msg);

  msg.expect_update_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=4;
  msg.expect_reset_calls.push_back(msg.server_id);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


TEST(InteractiveMarkerClient, init_twoservers)
{
  Msg msg;

  std::vector<Msg> seq;

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server1";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.expect_update_seq_num.push_back(1);
  seq.push_back(msg);

  msg.expect_update_seq_num.clear();

  msg.type=Msg::INIT;
  msg.seq_num=0;
  msg.server_id="server2";
  msg.frame_id=target_frame;
  seq.push_back(msg);

  msg.type=Msg::KEEP_ALIVE;
  msg.expect_init_seq_num.push_back(0);
  seq.push_back(msg);

  msg.expect_init_seq_num.clear();

  msg.type=Msg::UPDATE;
  msg.seq_num=1;
  msg.expect_update_seq_num.push_back(1);
  seq.push_back(msg);

  SequenceTest t;
  t.test(seq);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "im_client_test");
  //ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
