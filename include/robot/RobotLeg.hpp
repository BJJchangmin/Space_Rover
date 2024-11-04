/*! @file RobotLeg.hpp
 *  @brief Data structure containing parameters of a robot leg
 */

#ifndef MCL_ROBOT_LEG_HPP_
#define MCL_ROBOT_LEG_HPP_

// C++ standard library
#include <iostream>
#include <memory>
#include <vector>

// Eigen3
#include <eigen3/Eigen/StdVector>

// MuJoCo
#include <mujoco/mujoco.h>

// customized headers
#include "cppTypes.hpp"

using std::vector;

template <typename T>
class RobotLeg
{
//! Make Variable in here, declare definition in own Robotfile (Mclquad.hpp)
//! Receive sensor data in RobotLeg.cpp -> if change robot xml file, change sensor number
public:
  /**
   * * Model, Joint, Control에 관련된 변수들은 전부 여기에 선언
   * todo: Physical Parameter, Task Space Control Parameter는 제어할 방법 가닥이 나오면 추가
   * ! 여기서 사용할 변수들 정리하고, Robot.cpp 파일에서 sensor data setting, rover model file에서 hardware variable setting
   * @param PhysicalParameter = mass, leg_length, Link_com_location, Joint_location
   * @param Sensor_data = Body data, Joint data
   * @param ControlParameter = Gain_tunning, Task_space(Des,Act), Joint_space_Des(Des,Act)
   **/

  //* Physical parameters *//

  //* variables for control parameters *//
  // Mat2<T> jacbRW[4], jacbRW_inv[4];

  //! Be careful Vec3, Vec2 -> Because of HAA
  Vec2<T> kp_[4], kd_[4];
  Vec3<T> joint_pos_des_[4], joint_vel_des_[4], joint_torque_des_[4];

  //* variables for sensor data *//
  Vec3<T> body_pos_world_, body_vel_world_;
  Vec3<T> body_omega_world_;
  Vec4<T> body_ang_quat_world_;


  Vec4<T> joint_pos_act_[4], joint_vel_act_[4];

  T foot_contact_[4];
  Vec3<T> foot_grf_world_[4];


  //* variables for StanceForceControl *//

  //* Variables for Compensation Control *//

  //************************************* CONSTANTS ************************************************
  static constexpr size_t k_num_joint = 4;     // num of leg joints
  static constexpr size_t k_num_dof_body = 6;  // num of body DoF

  //************************************* METHODS **************************************************
  void get_sensor_data(mjData * data);

};

#endif  // MCL_ROBOT_LEG_HPP_