#include "MotorController.hpp"
#include "orientation_tools.hpp"
#include "Useful.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
MotorController<T>::MotorController(RobotLeg<T> & robot) : robot_(robot)
{
  foot_traj_ptr_ = nullptr;
  joint_traj_ptr_ = nullptr;


  for (size_t i = 0; i < 4; i++)
  {
    J_sus_[i] = 0.04; J_steer_[i] = 0.3; J_drive_[i] = 0.09;
    B_sus_[i] = 0.0101; B_steer_[i] = 0.0084; B_drive_[i] = 0.0034;


    error_sus_[i][0] = 0; error_sus_[i][1] = 0;
    error_dot_sus_[i][0] = 0; error_dot_sus_[i][1] = 0;
    error_int_sus_[i][0] = 0; error_int_sus_[i][1] = 0;
    pid_out_sus_[i][0] = 0; pid_out_sus_[i][1] = 0;

    error_steer_[i][0] = 0; error_steer_[i][1] = 0;
    error_dot_steer_[i][0] = 0; error_dot_steer_[i][1] = 0;
    error_int_steer_[i][0] = 0; error_int_steer_[i][1] = 0;
    pid_out_steer_[i][0] = 0; pid_out_steer_[i][1] = 0;

    error_drive_[i][0] = 0; error_drive_[i][1] = 0;
    error_dot_drive_[i][0] = 0; error_dot_drive_[i][1] = 0;
    error_int_drive_[i][0] = 0; error_int_drive_[i][1] = 0;
    pid_out_drive_[i][0] = 0; pid_out_drive_[i][1] = 0;

  }
}
//******************** TOTAL MOTOR CONTROLLER ********************/
template <typename T>
void MotorController<T>::drive_motor_control()
{
  M_PID_drive();

  for(size_t i = 0; i < 4; i++)
  {
    robot_.joint_torque_des_[i][2] = saturation_block(12.8, -12.8, pid_out_drive_[i][0]);

    pid_out_drive_[i][1] = pid_out_drive_[i][0];
  }

}

template <typename T>
void MotorController<T>::steer_motor_control()
{
  M_PID_steer();
  for(size_t i = 0; i < 4; i++)
  {
    robot_.joint_torque_des_[i][1] = 200*saturation_block(9.87,-9.87, pid_out_steer_[i][0]);

    pid_out_steer_[i][1] = pid_out_steer_[i][0];
  }


}

template <typename T>
void MotorController<T>::suspension_motor_control()
{
  M_PID_sus();
  for(size_t i = 0; i < 4; i++)
  {
    robot_.joint_torque_des_[i][0] = 498*saturation_block(44.82,-44.82, pid_out_sus_[i][0]);

    pid_out_sus_[i][1] = pid_out_sus_[i][0];
  }
}

//******************** PID MOTOR CONTROLLER ********************/
template <typename T>
void MotorController<T>::M_PID_drive()
{
  //Drive Motor Control
  Vec4 <T> ctrl_input;
  Vec4 <T> track_error; // Variable for Anti-Windup


  Vector3d K[4];

  for (size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    K[i] = PI_Gain_velctrl(J_drive_[i], B_drive_[i], 1.5, 5);

    // Error Calculation
    error_drive_[i][0] = joint_traj_ptr_->joint_vel_des_[i][2] - robot_.joint_vel_act_[i][3];

    //********** */ Calculate for Anti-windup ******************************************** */
    //todo: Anti-windup on/off 기능 추가
    track_error[i] = pid_out_drive_[i][1] - robot_.joint_torque_des_[i][2];

    //************************************ */

    error_drive_anti_[i][0] = Anti_Windup(0.1, track_error[i], error_drive_[i][0]);
    error_int_drive_[i][0] = integrate(error_drive_anti_[i][0], error_drive_anti_[i][1], error_int_drive_[i][1]);

    pid_out_drive_[i][0] = K[i][0]* error_drive_[i][0] + K[i][1] * error_int_drive_[i][0];


    // Error Update
    error_drive_[i][1] = error_drive_[i][0];
    error_drive_anti_[i][1] = error_drive_anti_[i][0];
    error_dot_drive_[i][1] = error_dot_drive_[i][0];
    error_int_drive_[i][1] = error_int_drive_[i][0];
  }
}

template <typename T>
void MotorController<T>::M_PID_steer()
{
  //Steer Motor Control
  Vector3d K[4];

  for(size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    K[i] = PD_Gain_posctrl(J_steer_[i], B_steer_[i], 1.5, 15);

    // Error Calculation
    error_steer_[i][0] = joint_traj_ptr_->joint_pos_des_[i][1] - robot_.joint_pos_act_[i][2];

    error_dot_steer_[i][0] = tustin_derivative(error_steer_[i][0], error_steer_[i][1], error_dot_steer_[i][1], 100);

    // Control Input
    pid_out_steer_[i][0] = K[i][0] * error_steer_[i][0] + K[i][2] * error_dot_steer_[i][0];

    // Error Calculation
    error_steer_[i][1] = error_steer_[i][0];
    error_dot_steer_[i][1] = error_dot_steer_[i][0];
    error_int_steer_[i][1] = error_int_steer_[i][0];

  }
}

template <typename T>
void MotorController<T>::M_PID_sus()
{
  // Suspension Motor Control
  Vector3d K[4];
  for(size_t i = 0; i < 4; i++)
  {
    // Gain Setting
    K[i] = PD_Gain_posctrl(J_sus_[i], B_sus_[i], 1.5, 15);

    // Error Calculation
    error_sus_[i][0] = joint_traj_ptr_->joint_pos_des_[i][0] - robot_.joint_pos_act_[i][0];

    error_dot_sus_[i][0] = tustin_derivative(error_sus_[i][0], error_sus_[i][1], error_dot_sus_[i][1], 100);

    // Control Input
    pid_out_sus_[i][0] = K[i][0] * error_sus_[i][0] + K[i][2] * error_dot_sus_[i][0];

    // Error Update
    error_sus_[i][1] = error_sus_[i][0];
    error_dot_sus_[i][1] = error_dot_sus_[i][0];
    error_int_sus_[i][1] = error_int_sus_[i][0];
  }
}


//******************** POINTER SETTING ********************/
template <typename T>
void MotorController<T>::get_traj_pointer(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr)
{

    foot_traj_ptr_ = foot_traj_ptr;
    joint_traj_ptr_ = joint_traj_ptr;

}

template class MotorController<float>;
template class MotorController<double>;

