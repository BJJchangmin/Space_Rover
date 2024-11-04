#ifndef MCL_MUJOCO_INTERFACE_HPP_
#define MCL_MUJOCO_INTERFACE_HPP_

//* C++ standard library *//
#include <thread>

//* MuJoCo headers *//
// #include <mujoco/mujoco.h>
#include "simulate.h"

//* customized headers *//
#include "Mclquad.hpp"
#include "RobotLeg.hpp"

namespace mj = ::mujoco;


template <typename T>
class MuJoCoInterface
{
private:
  mjData * data_;
  mjModel * model_;
  mj::Simulate * sim_;
  // std::thread interface_thread_;

  RobotLeg<T> & robot_;

public:
  // friend class RobotLeg<float>;
  explicit MuJoCoInterface(mj::Simulate * sim, RobotLeg<T> robot);

  struct MuJoCoActuatorCommand
  {
    /**
     * @brief Actuator Command
     * ! FL=0, FR=1, RL=2, RR=3
     * ! 0: HAA, 1: HFE, 2: KFE
     */

    Vec3<T> kp_[4];
    Vec3<T> kd_[4];
    Vec3<T> joint_pos_des_[4];
    Vec3<T> joint_vel_des_[4];
    Vec3<T> joint_torque_des_[4];
  };

  std::shared_ptr<MuJoCoActuatorCommand> actuator_cmd_ptr_;
  std::shared_ptr<MuJoCoActuatorCommand> get_actuator_cmd_ptr();

};

#endif // MCL_MUJOCO_INTERFACE_HPP_
