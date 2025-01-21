
#ifndef DATA_LOGGING_HPP_
#define DATA_LOGGING_HPP_

// C++ standard libraries
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "MotionTrajectory.hpp"
#include "Mclquad.hpp"
#include "estimate.hpp"


template <typename T>
class DataLogging
{

  private:
    std::ofstream fout_[5];
    static constexpr int k_save_sampling = 1;  // sampling rate at which data is saved
    RobotLeg<T> & robot_;

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;
    std::shared_ptr<typename Estimate<T>::EstimateParam> estimate_param_ptr_;


  public:
    explicit DataLogging(RobotLeg<T> & robot);

    void init_data();
    void save_data(const mjModel * m, mjData * d);

    void get_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

    void get_estimate_ptr( std::shared_ptr<typename Estimate<T>::EstimateParam> estimate_param_ptr);

    int get_logging_freq();

};




#endif  // data_logging_HPP_
