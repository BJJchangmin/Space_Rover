#include "estimate.hpp"
#include "orientation_tools.hpp"
#include "Useful.hpp"

#include <iostream>

using namespace ori;
using namespace std;

template <typename T>
Estimate<T>::Estimate(RobotLeg<T> & robot) : robot_(robot)
{
  estimate_param_ptr_ = std::make_shared<EstimateParam>();
}

template <typename T>
void Estimate<T>::slip_ratio_estimate()
{
  T radius = 0.15;
  T Rover_vel_x;

  Rover_vel_x = Max(robot_.body_vel_world_[0],0.001); // 초기 속도가 없을 때 0으로 나누는 것을 방지하기 위함

  for(size_t i = 0; i < 4; i++)
  {
    wheel_vel_[i] = radius*(robot_.joint_vel_act_[i][3]); // reference에 반영이 되어 있어서 기어비 고려안해줘도 됨

    estimate_param_ptr_->slip_ratio_[i]
      = abs((Rover_vel_x - wheel_vel_[i])/Rover_vel_x);

    // cout <<"slip_ratio : " << estimate_param_ptr_->slip_ratio_[0] << endl;
    estimate_param_ptr_->slip_ratio_[i] = saturation_block(1, 0, estimate_param_ptr_->slip_ratio_[i]); // 일단 전진만 한다는 가정
  }
  // cout << "Rover_vel_x : " << Rover_vel_x << endl;
  // cout << "wheel_vel : " << wheel_vel_[0] << endl;
}

template <typename T>
void Estimate<T>::cal_mu()
{
  /**
   * @brief : Lambda-mu 곡선 완벽히 구현 X 일단 stribeck model로 최대한 비슷하게 구현
   * @brief : Graph가 어떻게 그려지는 지는 Data 폴더 내의 Mu_Lambda.m 파일 참고
   */
  T e,T_brk,T_c,w_brk,w_st,w_coul,f,bias;
  e = 2.71828;
  T_brk = 0.8;
  T_c = 0.5;
  w_brk = 0.4;
  w_st = sqrt(2)*w_brk;
  w_coul = w_brk/10;
  f = -0.01;
  bias = 0;
  T S[4];
  cout << "slip_ratio : " << estimate_param_ptr_->slip_ratio_[0] << endl;
  slip_ratio_estimate();
  for(size_t i = 0; i < 4; i++)
  {
    S[i] = estimate_param_ptr_->slip_ratio_[i];

    estimate_param_ptr_->mu_[i] = sqrt(2)*e*(T_brk - T_c)*exp(-pow(S[i]/w_st,2))*(S[i]/ w_st) +
      T_c* tanh(S[i]/ w_coul) + f*S[i] + bias;

    estimate_param_ptr_->mu_[i] = max(0.01, estimate_param_ptr_->mu_[i]);

  }

  cout << "mu : " << estimate_param_ptr_->mu_[0] << endl;

}

template <typename T>
std::shared_ptr<typename Estimate<T>::EstimateParam> Estimate<T>::set_estimate_param_ptr()
{
  return estimate_param_ptr_;
}




template class Estimate<float>;
template class Estimate<double>;
