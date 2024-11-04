#include "data_logging.hpp"

using namespace std;

/**
 * ! If you don't change file name, and run the simulation again, the data will be overwritten.
 */
template <typename T>
DataLogging<T>::DataLogging(RobotLeg<T> & robot)
  : fout_FL_("../data/data_FL.csv"),
    fout_FR_("../data/data_FR.csv"),
    fout_RL_("../data/data_RL.csv"),
    fout_RR_("../data/data_RR.csv"),
    fout_trunk_("../data/data_trunk.csv"),
    robot_(robot)
{
  cout << "Data Logger object is created" << endl;



  foot_traj_ptr_ = nullptr;

  if (!fout_FL_)
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }

  init_data_FL();

}

/**
 * @brief Select data to save
 * You can select which data of the LoggingData struct to save.
 * * Data are separated by a comma (,) followed by a space
 * ! comma (,) should be omitted in the last line and newline must exist after the last data.
 */
template <typename T>
void DataLogging<T>::save_data_FL(const mjModel* m, mjData* d)
{
  if (!fout_FL_)
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }
  else
  {
    // // Debugging: Print to console what we're writing
    // std::cout << "Writing data to file: " << std::endl;
    // std::cout << d->time << ", " << robot_.foot_pos_rw_act_local_[0] << ", "
    //           << robot_.foot_pos_rw_act_local_[1] << ", " << robot_.foot_pos_rw_act_local_[2] << ", "
    //           << robot_.foot_vel_rw_act_local_[0] << std::endl;

    fout_FL_ << d->time ; // time




    // ! Don't remove the newline
    fout_FL_ << endl;

    }

}

/**
 * @brief Set names of data to be saved
 * After you select which data to save, you can set the headers here.
 * You have to set the headers in the same order as the data and it is recommended that
 * the headers should be named as brief but you can recognize the data.
 * * Data are separated by a comma (,) followed by a space
 */
template <typename T>
void DataLogging<T>::init_data_FL()
{
  if (!fout_FL_)
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }
  else
  {
    fout_FL_ << "time" << std::endl;
  }
}

template <typename T>
void DataLogging<T>::set_traj_ptr(
    std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> foot_traj_ptr,
    std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> joint_traj_ptr)
{
  foot_traj_ptr_ = foot_traj_ptr;
  joint_traj_ptr_ = joint_traj_ptr;
}


template <typename T>
int DataLogging<T>::get_logging_freq()
{
  return k_save_sampling;
}

template class DataLogging<float>;
template class DataLogging<double>;
