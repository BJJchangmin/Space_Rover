#include "TrackingController.hpp"
#include "orientation_tools.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
TrackingController<T>::TrackingController(RobotLeg<T> & robot) : robot_(robot)
{
  foot_traj_ptr_ = nullptr;
  joint_traj_ptr_ = nullptr;

  for (size_t i = 0; i < 4; i++)
  {

    kp_[i][0] = 100;
    kp_[i][1] = 10;

  }


}

template <typename T>
void TrackingController<T>::drive_motor_control()
{
  //dirve motor 번호 받아오기
  T error_vel_[4];
  for (size_t i = 0; i < 4; i++)
  {
    error_vel_[i] = joint_traj_ptr_->joint_vel_des_[i][2] - robot_.joint_vel_act_[i][3];
    robot_.joint_torque_des_[i][2] = kp_[i][1] * error_vel_[i];
  }

}

template <typename T>
void TrackingController<T>::suspension_motor_control()
{
  // suspension
  T error_pos_[4];

  for(size_t i = 0; i < 4; i++)
  {
    error_pos_[i] = joint_traj_ptr_->joint_pos_des_[i][0] - robot_.joint_pos_act_[i][0];
    robot_.joint_torque_des_[i][0] = kp_[i][0] * error_pos_[i];
  }
}


template <typename T>
void TrackingController<T>::get_traj_pointer(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr)
{

    foot_traj_ptr_ = foot_traj_ptr;
    joint_traj_ptr_ = joint_traj_ptr;

}

template class TrackingController<float>;
template class TrackingController<double>;

