
#include "RobotLeg.hpp"
#include "orientation_tools.hpp"

using namespace ori;
using namespace std;

template <typename T>
void RobotLeg<T>::get_sensor_data(mjData * data)
{
  //* get body position *//
  for (size_t i = 0; i< 3; i++)
  {
    body_pos_world_[i] = data->sensordata[i + 12];
    body_vel_world_[i] = data->sensordata[i + 6];
    body_omega_world_[i] = data->sensordata[i + 9];
  }

  for (size_t i = 0; i < 4; i++)
  {
    body_ang_quat_world_[i] = data->sensordata[i + 15];
    foot_contact_[i] = data->sensordata[31 + 4*i];

    for(size_t j = 0; j < 3; j++)
    {
      foot_grf_world_[i][j] = data->sensordata[19 + 4*i + j];
    }

    for (size_t j = 0; j < 4; j++)
    {
      // [4][4] = [FL FR RL RR][sus, sus_cover, steer, drive] -> 순서
      joint_pos_act_[i][j] = data->qpos[7 + 4*i + j];
      joint_vel_act_[i][j] = data->qvel[6 + 4*i + j];
    }

  }
}


template class RobotLeg<double>;
template class RobotLeg<float>;
