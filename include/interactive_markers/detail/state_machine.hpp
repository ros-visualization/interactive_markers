// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: David Gossow

#ifndef INTERACTIVE_MARKERS__DETAIL__STATE_MACHINE_HPP_
#define INTERACTIVE_MARKERS__DETAIL__STATE_MACHINE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace interactive_markers
{

// Helper class for state management
template<class StateT>
class StateMachine
{
public:
  StateMachine(std::string name, StateT init_state);
  StateMachine & operator=(StateT state);
  operator StateT();
  rclcpp::Duration getDuration();

private:
  StateT state_;
  rclcpp::Clock clock_;
  rclcpp::Time chg_time_;
  std::string name_;
};

template<class StateT>
StateMachine<StateT>::StateMachine(std::string name, StateT init_state)
: state_(init_state),
  clock_(),
  chg_time_(clock_.now()),
  name_(name)
{
}

template<class StateT>
StateMachine<StateT> & StateMachine<StateT>::operator=(StateT state)
{
  if (state_ != state) {
    RCUTILS_LOG_DEBUG("Setting state of %s to %lu", name_.c_str(), (int64_t)state);
    state_ = state;
    chg_time_ = clock_.now();
  }
  return *this;
}

template<class StateT>
rclcpp::Duration StateMachine<StateT>::getDuration()
{
  return clock_.now() - chg_time_;
}

template<class StateT>
StateMachine<StateT>::operator StateT()
{
  return state_;
}

}  // namespace interactive_markers

#endif  // INTERACTIVE_MARKERS__DETAIL__STATE_MACHINE_HPP_
