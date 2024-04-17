#ifndef CONTROL_SYSTEM_PI
#define CONTROL_SYSTEM_PI

#include "Math_functions.h"

class PI_controller {
public:
	PI_controller();
	void init(double dt_, double Kp_, double Ki_, double u_k_1_, double I_max_);
	void set_param(double dt_, double Kp_, double Ki_, double u_k_1_, double I_max_);
	void set_dt(double dt_);
	void set_Kp(double Kp_);
	void set_Ki(double Ki_);
	void set_u_0(double u_k_1_);
	void set_I_max(double I_max_);
	double get_dt();
	double get_Kp();
	double get_Ki();
	double get_u_k_1();
	double get_I_max();
	double calc_u(double e_k);
	void reset();
	void merge(double u_k_1_);

private:
	Math_functions math_fun;
	double dt = 0.0;
	double u_k_1 = 0.0;
	double e_k_1 = 0.0;
	double Kp = 0.0;
	double Ki = 0.0;
	double I_max = 9999999999;
	bool start = true;
};

#endif