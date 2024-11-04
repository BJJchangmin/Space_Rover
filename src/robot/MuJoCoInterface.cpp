#include "MuJoCoInterface.hpp"

#include <unistd.h>
#include <iostream>

template <typename T>
MuJoCoInterface<T>::MuJoCoInterface(mj::Simulate * sim, RobotLeg<T> robot)
  : sim_(sim), robot_(robot)
{
  sim->run = false;
  sim_->ui0_enable = true;
  sim_->ui1_enable = true;
  sim_->pending_.load_key = true;

  actuator_cmd_ptr_ = std::make_shared<MuJoCoActuatorCommand>();
}

template <typename T>
std::shared_ptr<typename MuJoCoInterface<T>::MuJoCoActuatorCommand> MuJoCoInterface<T>::get_actuator_cmd_ptr()
{
  return actuator_cmd_ptr_;
}

template class MuJoCoInterface<float>;
template class MuJoCoInterface<double>;

