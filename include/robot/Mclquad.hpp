
#ifndef MCL_ROBOT_MCLQUAD_HPP_
#define MCL_ROBOT_MCLQUAD_HPP_

#include "RobotLeg.hpp"
#include "orientation_tools.hpp"

using namespace ori;

template <typename T>
RobotLeg<T> buildMclQuad()
{
  RobotLeg<T> mclQuad;
  
  // ! FL=0, FR=1, RL=2, RR=3
  
  // Physical parameters
  for(int i=0; i<4;i++)
  {
    // mclQuad.hip_mass_[i] = 1.047;
    // mclQuad.thigh_mass_[i] = 0.50316;
    // mclQuad.shank_mass_[i] = 0.28735;

    // mclQuad.thigh_link_length_[i] = 0.25;
    // mclQuad.shank_link_length_[i] = 0.25;

    // mclQuad.thigh_body_inertia_[i] << 2.6521E-04, 0.006258718, 0.006208038;
    // mclQuad.shank_body_inertia_[i] << 1.6488E-04, 0.003937, 0.004042;

    // mclQuad.thigh_com_location_[i] << 0.085604, 0, 0;
    // mclQuad.shank_com_location_[i] << 0.14427, 0, 0;

  }
  
  //* RW parameters *//
  for(int i=0; i<4;i++)
  {
    // mclQuad.leg_length_max_[i] = 0.45;
    // mclQuad.leg_length_min_[i] = 0.22;
    // mclQuad.leg_length_home_[i] = 0.32355;
    
    // //*ws parameters
    // mclQuad.swept_angle_max_[i] = deg2rad(60.0f);
    // mclQuad.swept_angle_min_[i] = deg2rad(-60.0f);
    // mclQuad.swept_angle_home_[i] = 0.0;
    // mclQuad.foot_contact_threshold_[i] = 25.0;
    

    
  }
  return mclQuad;

}

#endif // MCL_ROBOT_MCLQUAD_HPP_