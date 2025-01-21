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

    T Driving_Gear_Ratio;

    //* ID Parameters *//
    int NUM_SEGMENTS;
    int SEGMENT_DURATION;
    T freqs[20];
    T amps[20];
    T phases[20];
    T start_time;
    float shreder_ID_data_[30000];
    int signal_idx;

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
         * ! Driving Reference는 휠단에서 모터 input으로 바꿔줘야함
         */
        Vec3<T> joint_pos_des_[4];
        Vec3<T> joint_vel_des_[4];
        Vec3<T> joint_torque_des_[4];
    };

    std::shared_ptr<DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<DesiredFootTrajectory> set_foot_traj_ptr();

    std::shared_ptr<DesiredJointTrajectory> joint_traj_ptr_;
    std::shared_ptr<DesiredJointTrajectory> set_joint_traj_ptr();

  public:

    MotionTrajectory();
    void squat(T time);
    void driving_test(T time);
    void suspension_test(T time);

    void RandomFreqs();
    void loadDataFromFile(const char* filename);
    T motor_ID(T time);
    T system_ID(T time);



};

#endif  // MOTION_TRAJECTORY_HPP_
