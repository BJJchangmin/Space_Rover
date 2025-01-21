
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
    //******************** 기어비 반영 부분 **********************/
    joint_pos_act_[i][0] = data->qpos[7 + 4*i]/498;
    joint_vel_act_[i][0] = data->qvel[6 + 4*i]/498;

    joint_pos_act_[i][1] = data->qpos[8 + 4*i]/498;
    joint_vel_act_[i][1] = data->qvel[7 + 4*i]/498;

    joint_pos_act_[i][2] = data->qpos[9 + 4*i]/200;
    joint_vel_act_[i][2] = data->qvel[8 + 4*i]/200;

    joint_pos_act_[i][3] = data->qpos[10 + 4*i]; // 기어비 반영은 Motor Reference에서 해줄 예정
    joint_vel_act_[i][3] = data->qvel[9 + 4*i];


  }
}


template class RobotLeg<double>;
template class RobotLeg<float>;
