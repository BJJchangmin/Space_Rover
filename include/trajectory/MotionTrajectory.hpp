#ifndef MOTION_TRAJECTORY_HPP_
#define MOTION_TRAJECTORY_HPP_

#include <mujoco/mujoco.h>
#include <memory>

#include "Mclquad.hpp"
#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class MotionTrajectory
{
  private:
    T t_;
    T T_pause_;   // pause duration
    T T_crouch_;  // crouch duration
    // T f_crouch_;  // crouch frequency = 1 / T_crouch_
    T T_land_;     // landing duration
    T T_recover_;  // recover duration

    T h_max_, h_min_, h_home_;  // max/min and home height of body

    // parameter for Test StanceForceControl
    T dthr_init_;

    T state_;

  public:
    struct DesiredFootTrajectory
    {
        Vec2<T> foot_pos_rw_des_[4];
        Vec2<T> foot_vel_rw_des_[4];

    };

    struct DesiredJointTrajectory
    {
        /**
         * @param joint_des_: [FL FR RL RR][Sus, Steer, Drive] -> [4][3] 이 순서다
         */
        Vec3<T> joint_pos_des_[4];
        Vec3<T> joint_vel_des_[4];
        Vec3<T> joint_torque_des_[4];
    };

    std::shared_ptr<DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<DesiredFootTrajectory> get_foot_traj_ptr();

    std::shared_ptr<DesiredJointTrajectory> joint_traj_ptr_;
    std::shared_ptr<DesiredJointTrajectory> get_joint_traj_ptr();

  public:

    MotionTrajectory();
    void squat(T time);
    void driving_test();


};

#endif  // MOTION_TRAJECTORY_HPP_
