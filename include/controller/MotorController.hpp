#ifndef POS_CONTROLLER_HPP_
#define POS_CONTROLLER_HPP_

#include "MotionTrajectory.hpp"
#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class MotorController
{
  private:
    RobotLeg<T> & robot_;
    //** Motor Model */
    T J_sus_[4], J_steer_[4], J_drive_[4]; // Motor Inertia from Identification
    T B_sus_[4], B_steer_[4], B_drive_[4]; // Damping from Identification

    //** PID Control Variable */
    Vec2<T> error_sus_[4], error_dot_sus_[4], error_int_sus_[4], pid_out_sus_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> error_steer_[4], error_dot_steer_[4], error_int_steer_[4], pid_out_steer_[4]; // [i][0] : Current, [i][1] : Old
    Vec2<T> error_drive_[4], error_drive_anti_[4], error_dot_drive_[4], error_int_drive_[4], pid_out_drive_[4]; // [i][0] : Current, [i][1] : Old

    //** Disturbance Observer Variable */



    //** Motor Trajectory Variable */
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;

  public:
    MotorController(RobotLeg<T> & robot);

    void get_traj_pointer(
        std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
        std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

    void M_PID_drive(); // Drive Motor PID Control
    void M_PID_steer(); // Steer Motor PID Control
    void M_PID_sus(); // Suspension Motor PID Control

    void drive_motor_control(); // Drive Motor Control(PID+FF+DOB)
    void suspension_motor_control(); // Suspension Motor Control(PID+FF+DOB)
    void steer_motor_control(); // Steer Motor Control(PID+FF+DOB)






};


#endif  // POS_CONTROLLER_HPP_
