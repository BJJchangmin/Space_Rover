#include "Useful.hpp"

using namespace Eigen;

double Ts = 0.001;
double pi = 3.141592;

double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double integrate(double input, double input_old, double output_old)
{
  double output = 0;
  output = (Ts/2)*(input + input_old) + output_old;
  return output;

}

double LPF(double input, double input_old, double output_old, double cutoff_freq)
{
    /**
     * @brief LPF
     * Filter = 1 / (tau*s + 1)
     */
    double time_const = 1 / (2 * pi * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double Second_order_LPF(double input, double input_old, double input_old2, double output_old, double output_old2, double zeta, double cutoff_freq)
{
  /**
   * @brief Second_order_LPF
   * Filter = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
   */
  double wn = 2 * pi * cutoff_freq;
  double a = pow(wn,2)*pow(Ts,2)*(input + 2*input_old + input_old2);
  double y_param = 4 + 4*zeta*wn*Ts + pow(wn*Ts,2);
  double y_old_param = 2*pow(wn*Ts,2) - 8;
  double y_old2_param = 4 - 4*zeta*wn*Ts + pow(wn*Ts,2);
  double output = (a - y_old_param*output_old - y_old2_param*output_old2) / y_param;

  return output;
}

double saturation_block(double up_limit, double low_limit, double input)
{
 if(input > up_limit)
 {
    return up_limit;
 }
 else if(input < low_limit)
 {
    return low_limit;
 }
 else
 {
    return input;
 }
}

double Max(double input, double compare)
{
  if(input > compare)
  {
    return input;
  }
  else
  {
    return compare;
  }
}

double dot(double input, double input_old,double output_old)
{
  double output = 0;
  output = (2/Ts)*(input - input_old) - output_old;
  return output;

}

Vector3d PD_Gain_posctrl(double J, double B, double zeta, double W_n) // J, B, zeta, W_n
{
    //! Gain Calculate by Pole Placement
    //* J : Inertia
    //* B : Damping
    //* zeta : Damping Ratio
    //* W_n : Natural Frequency
    //* K[0] : Kp, K[1] : Ki, K[2] : Kd

    Vector3d K;
    W_n = W_n*2*pi;

    K[0] = J*W_n*W_n;
    K[1] = 0;
    K[2] = 2*zeta*W_n*J - B;
    return K;
}

Vector3d PI_Gain_velctrl(double J, double B, double zeta, double W_n) // J, B, zeta, W_n
{
    //! Gain Calculate by Pole Placement
    //* J : Inertia
    //* B : Damping
    //* zeta : Damping Ratio
    //* W_n : Natural Frequency
    //* K[0] : Kp, K[1] : Ki, K[2] : Kd

    Vector3d K;
    W_n = W_n*2*pi;

    K[0] = 2*zeta*W_n*J - B; //Kp
    K[1] = J*W_n*W_n; //Ki
    K[2] = 0; //Kd
    return K;
}

double Rate_Limit(double input, double input_old, double rate_limit)
{
  double new_input;
  double delta = input - input_old;
  double max_change = rate_limit * Ts;

  if (delta > max_change)
  {
    delta = max_change;
  }
  else if (delta < -max_change)
  {
    delta = -max_change;
  }
  else
  {
    delta = delta;
  }

  return new_input = input_old + delta;

}

double Anti_Windup(double gain, double ctrl_error, double track_error)
{
  return gain * ctrl_error + track_error;
}


double DOB_hat(double ctrl_input, double Jn, double Bn)
{
  double Left ;
  double Right;
  double s;
  double d_hat;
  double d_hat_old;
  double LPF_d_hat_old;
  double LPF_d_hat;

  Left = ctrl_input; // PID old Value
  Right = Jn *s + Bn; // Motor Model

  d_hat = Left - Right;
  //d_hat_old = 변수[1step delay];
  //LPF_d_hat_old = 변수[1step delay];

  // 변수[현재값] = d_hat;


}

double FeedForward_1st_order(double command, double J, double B)
{
  double vel_cmd;
  double vel_cmd_dot;
  double vel_cmd_old;
  double vel_cmd_dot_old;
  double tau_ff;
  double tau_ff_old;
  double LPF_tau_ff_old;
  double LPF_filter;
  double LPF_tau_ff;

  /**
   * @brief FeedForward_1st_order
   * !주석에 해당하는 변수들 생성 필요
   * !주석에 해당하는 변수들을 전부 만들면 되고 변수명 변경 가능. 함수 전체를 들고 가는게 아니라 이 form을 사용
   * ! 함수 내에서 선언해준 변수와 사용하는 곳에서의 변수 숫자가 같아야함
   */

  // Traj변수[i][1] = Traj변수[i][0];         vel_cmd_old = Traj변수[i][1];
  // Traj변수[i][0] = command;               vel_cmd = Traj변수[i][0];
  // cmd_dot변수[i][1] = cmd_dot변수[i][0];   vel_cmd_dot_old = cmd_dot변수[i][1];
  // tau_ff변수[i][1] = tau_ff변수[i][0];     tau_ff_old = tau_ff변수[i][1];
  // LPF_tau_ff변수[i][1] = LPF_tau_ff변수[i][0]; LPF_tau_ff_old = LPF_tau_ff변수[i][1];

  // LPF_filter = LPF 변수



  vel_cmd_dot = dot(vel_cmd, vel_cmd_old, vel_cmd_dot_old);
  // cmd_dot변수[i][0] = vel_cmd_dot;

  tau_ff = J * vel_cmd_dot + B;
  // tau_ff변수[i][0] = tau_ff;

  LPF_tau_ff = LPF(tau_ff, tau_ff_old, LPF_tau_ff_old, LPF_filter);
  // LPF_tau_ff변수[i][0] = LPF_tau_ff;

  return LPF_tau_ff;

}

double FeedForward_2nd_order(double command, double J, double B, double K)
{
  double pos_cmd;
  double pos_cmd_dot;
  double pos_cmd_2dot;
  double pos_cmd_old;
  double pos_cmd_dot_old;
  double pos_cmd_2dot_old;
  double tau_ff;
  double tau_ff_old;
  double tau_ff_old2;
  double LPF_tau_ff_old;
  double LPF_tau_ff_old2;
  double LPF_tau_ff;
  double LPF_zeta;
  double LPF_filter;

  // Traj변수[i][1] = Traj변수[i][0];         pos_cmd_old = Traj변수[i][1];
  // Traj변수[i][0] = command;               pos_cmd = Traj변수[i][0];

  //cmd_dot변수[i][1] = cmd_dot변수[i][0];   pos_cmd_dot_old = cmd_dot변수[i][1];
  //cmd_2dot변수[i][1] = cmd_2dot변수[i][0]; pos_cmd_2dot_old = cmd_2dot변수[i][1];

  // tau_ff변수[i][2] = tau_ff변수[i][1];     tau_ff_old2 = tau_ff변수[i][2];
  // tau_ff변수[i][1] = tau_ff변수[i][0];     tau_ff_old = tau_ff변수[i][1];

  // LPF_tau_ff변수[i][2] = LPF_tau_ff변수[i][1]; LPF_tau_ff_old2 = LPF_tau_ff변수[i][2];
  // LPF_tau_ff변수[i][1] = LPF_tau_ff변수[i][0]; LPF_tau_ff_old = LPF_tau_ff변수[i][1];

  // LPF_filter = wn
  // LPF_zeta = zeta

  pos_cmd_dot = dot(pos_cmd, pos_cmd_old, pos_cmd_dot_old);
  // cmd_dot변수[i][0] = pos_cmd_dot;

  pos_cmd_2dot = dot(pos_cmd_dot,pos_cmd_dot_old, pos_cmd_2dot_old);
  // cmd_2dot변수[i][0] = pos_cmd_2dot;

  tau_ff = J * pos_cmd_2dot + B * pos_cmd_dot + K * pos_cmd;
  // tau_ff변수[i][0] = tau_ff;

  LPF_tau_ff = Second_order_LPF(tau_ff, tau_ff_old, tau_ff_old2,LPF_tau_ff_old, LPF_tau_ff_old2, LPF_zeta, LPF_filter);
  // LPF_tau_ff변수[i][0] = LPF_tau_ff;

  return LPF_tau_ff;

}



