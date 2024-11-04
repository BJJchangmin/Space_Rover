#ifndef POS_CONTROLLER_HPP_
#define POS_CONTROLLER_HPP_

#include "MotionTrajectory.hpp"
#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class TrackingController
{
  private:
    RobotLeg<T> & robot_;
    Vec2<T> kp_[4], kd_[4];

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;

  public:
    TrackingController(RobotLeg<T> & robot);

    void get_traj_pointer(
        std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
        std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

  void drive_motor_control();
  void suspension_motor_control();





};


#endif  // POS_CONTROLLER_HPP_
