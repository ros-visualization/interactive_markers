/*
 * state_machine.h
 *
 *  Created on: Jul 17, 2012
 *      Author: gossow
 */

#ifndef INTERACTIVE_MARKERS_STATE_MACHINE_H_
#define INTERACTIVE_MARKERS_STATE_MACHINE_H_

#include <ros/ros.h>

namespace interactive_markers
{

// Helper class for state management
template<class StateT>
class StateMachine
{
public:
  StateMachine( std::string name, StateT init_state );
  StateMachine& operator=( StateT state );
  operator StateT();
  ros::Duration getDuration();
private:
  StateT state_;
  ros::Time chg_time_;
  std::string name_;
};

template<class StateT>
StateMachine<StateT>::StateMachine( std::string name, StateT init_state )
: name_(name)
{
  operator=(init_state);
};

template<class StateT>
StateMachine<StateT>& StateMachine<StateT>::operator=( StateT state )
{
  if ( state_ != state )
  {
    ROS_DEBUG( "Setting state of %s to %lu", name_.c_str(), (int64_t)state );
    state_ = state;
    chg_time_=ros::Time::now();
  }
  return *this;
}

template<class StateT>
ros::Duration StateMachine<StateT>::getDuration()
{
  return ros::Time::now()-chg_time_;
}

template<class StateT>
StateMachine<StateT>::operator StateT()
{
  return state_;
}

}

#endif /* STATE_MACHINE_H_ */
