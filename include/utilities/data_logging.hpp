
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


template <typename T>
class DataLogging
{

  private:
    enum LegID
    {
      FL = 0,
      FR = 1,
      RL = 2,
      RR = 3
    };

    std::ofstream fout_FL_;
    std::ofstream fout_FR_;
    std::ofstream fout_RL_;
    std::ofstream fout_RR_;
    std::ofstream fout_trunk_;
    static constexpr int k_save_sampling = 1;  // sampling rate at which data is saved
    RobotLeg<T> & robot_;

    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr_;
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr_;

  public:
    explicit DataLogging(RobotLeg<T> & robot);

    void init_data_FL();
    void init_data_FR();
    void init_data_RL();
    void init_data_RR();
    void init_data_trunk();

    void save_data_FL(const mjModel * m, mjData * d);
    void save_data_FR(const mjModel * m, mjData * d);
    void save_data_RL(const mjModel * m, mjData * d);
    void save_data_RR(const mjModel * m, mjData * d);
    void save_data_trunk(const mjModel * m, mjData * d);

   void set_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr);

    int get_logging_freq();



};




#endif  // data_logging_HPP_
