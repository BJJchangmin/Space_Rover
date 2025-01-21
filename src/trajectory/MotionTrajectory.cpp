#include "MotionTrajectory.hpp"

#include <cmath>
#include <iostream>
#include <random>

using namespace std;

template <typename T>
MotionTrajectory<T>::MotionTrajectory()
{
  Driving_Gear_Ratio = 40;

  NUM_SEGMENTS = 20;
  SEGMENT_DURATION = 20;
  signal_idx = 0;
  // loadDataFromFile("../data/ID_torque_input/input33_colvec.txt");
  // loadDataFromFile("../data/ID_torque_input/torque_input.txt");
  loadDataFromFile("../data/ID_torque_input/torque_input2.txt");

  foot_traj_ptr_ = std::make_shared<DesiredFootTrajectory>();
  joint_traj_ptr_ = std::make_shared<DesiredJointTrajectory>();
}

template <typename T>
void MotionTrajectory<T>::driving_test(T time)
{
  Vec4<T> Driving_M_Command;
  for(size_t i = 0; i < 4; i++)
  {
    //** Driving Test */
    if(time <= 10)
    {
      Driving_M_Command[i] = time/(20*Driving_Gear_Ratio);
    }
    else
    {
      Driving_M_Command[i] = 10.0/(20*Driving_Gear_Ratio);
    }


    joint_traj_ptr_->joint_pos_des_[i][0] = 0.0;//suspension Joint position reference

    joint_traj_ptr_->joint_pos_des_[i][1] = 0.0;//steer Joint position reference

    joint_traj_ptr_->joint_vel_des_[i][2] = Driving_M_Command[i]* Driving_Gear_Ratio; //drive Joint velocity reference


  }

}

template <typename T>
void MotionTrajectory<T>::suspension_test(T time)
{
  T f= 0.1;
  for(size_t i = 0; i < 4; i++)
  {
    //** Steering Test */
    // if(time <= 20)
    // {
    //   joint_traj_ptr_->joint_pos_des_[i][1] = 0.5*sin(2*M_PI*f*time); //steer Joint position reference
    // }
    // else if( 3 <= time <= 6)
    // {
    //   joint_traj_ptr_->joint_pos_des_[i][0] = 0.5*sin(2*M_PI*f*(time-3));
    // }

    //** Suspension Test */
    joint_traj_ptr_->joint_pos_des_[i][0] = 0.5*sin(2*M_PI*f*time);

  }
}

template <typename T>
T MotionTrajectory<T>::motor_ID(T time)
{
  start_time = 3;


  if (time < 0.003) // first time step
  {
    RandomFreqs();
  }
  // RandomFreqs();

  double ex_time = time - start_time;
  int current_segment = static_cast<int>(std::floor(ex_time / SEGMENT_DURATION));

  double seg_freq = freqs[current_segment];
  double seg_amp = amps[current_segment];
  double seg_phase = phases[current_segment];

  if (time < 3)
  {
    return 0;
  }
  else
  {
    double torque = seg_amp * sin(2 * M_PI * seg_freq * ex_time + seg_phase);
    return torque;
  }


}

template <typename T>
T MotionTrajectory<T>::system_ID(T time)
{
  double output;
  cout << "signal_idx: " << signal_idx << endl;
  if (time < 3 || signal_idx >= 30000)
  {
    return 0;
  }
  else
  {
    output = shreder_ID_data_[signal_idx];
    cout << "signal_idx: " << output << endl;
    signal_idx = signal_idx + 1;
    return output;

  }



}

template <typename T>
void MotionTrajectory<T>::RandomFreqs()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> freqDist(0.1, 100.0);
  std::uniform_real_distribution<double> ampDist(0.1, 0.5);
  std::uniform_real_distribution<double> phaseDist(0.0, 2.0 * M_PI);

  for (int i = 0; i < NUM_SEGMENTS; i++)
  {
    freqs[i] = freqDist(gen);
    amps[i] = ampDist(gen);
    phases[i] = phaseDist(gen);
  }

}

template <typename T>
void MotionTrajectory<T>::loadDataFromFile(const char* filename)
{
  double DATA_SIZE = 30000;

  FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Error opening file\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < DATA_SIZE; i++) {
        if (fscanf(file, "%f", &shreder_ID_data_[i]) != 1)
        {
          cout <<"hello" << endl;
          perror("Error opening file\n");
          fclose(file);
          exit(EXIT_FAILURE);
        }
    }

    fclose(file);

}


template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredFootTrajectory> MotionTrajectory<T>::set_foot_traj_ptr()
{
  return foot_traj_ptr_;
}

template <typename T>
std::shared_ptr<typename MotionTrajectory<T>::DesiredJointTrajectory> MotionTrajectory<T>::set_joint_traj_ptr()
{
  return joint_traj_ptr_;
}




template class MotionTrajectory<float>;
template class MotionTrajectory<double>;

