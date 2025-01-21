#ifndef ESTIMATE_HPP_
#define ESTIMATE_HPP_

#include "MuJoCoInterface.hpp"
#include "RobotLeg.hpp"

template <typename T>
class Estimate
{
  private:
    RobotLeg<T> & robot_;
    T wheel_vel_[4]; // [FL FR RL RR]


  public:
    struct EstimateParam
    {
      T slip_ratio_[4]; // [FL FR RL RR]
      double mu_[4]; // [FL FR RL RR]
    };

    std::shared_ptr<EstimateParam> estimate_param_ptr_;
    std::shared_ptr<EstimateParam> set_estimate_param_ptr();

  public:
    Estimate(RobotLeg<T> & robot);
    void slip_ratio_estimate();
    void cal_mu();
};

#endif  // ESTIMATE_HPP_
