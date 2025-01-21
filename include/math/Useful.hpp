#ifndef __FILTER_H__
#define __FILTER_H__


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;




double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);

double integrate(double input, double input_old, double output_old);

double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);

double saturation_block(double up_limit, double low_limit, double input);

double Max(double input, double compare);

double dot(double input, double input_old,double output_old);

Vector3d PD_Gain_posctrl(double J, double B, double zeta, double W_n);

Vector3d PI_Gain_velctrl(double J, double B, double zeta, double W_n);

double Rate_Limit(double input, double input_old, double rate_limit);

double Anti_Windup(double gain, double ctrl_error, double track_error);




#endif // !__FILTER_H__
