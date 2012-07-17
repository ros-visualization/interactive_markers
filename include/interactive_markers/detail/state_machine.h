/*
 * state_machine.h
 *
 *  Created on: Jul 17, 2012
 *      Author: gossow
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

namespace interactive_markers
{

// Helper class for state management
template<class StateT>
class StateMachine
{
public:
  StateMachine( StateT init_state );
  StateMachine& operator=( StateT state );
  operator StateT();
  ros::Duration getDuration();
private:
  StateT state_;
  ros::Time chg_time_;
};

template<class StateT>
StateMachine<StateT>& StateMachine<StateT>::operator=( StateT state )
{
  state_ = state;
  chg_time_=ros::Time::now();
  return *this;
}

template<class StateT>
StateMachine<StateT>::StateMachine( StateT init_state )
{
  operator=(init_state);
};

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
