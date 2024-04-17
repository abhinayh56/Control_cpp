#ifndef CONTROL_SYSTEM_D_LPF_1
#define CONTROL_SYSTEM_D_LPF_1

#include "Constants.h"

class D_LPF_1_controller {
public:
	D_LPF_1_controller();
	void init(double dt_, double Kd_, double fc_);
	void set_param(double dt_, double Kd_, double fc_);
	void set_dt(double dt_);
	void set_Kd(double Kd_);
	void set_fc(double fc_);
	double get_dt();
	double get_Kd();
	double get_fc();
	double get_e_k_1();
	double get_u_k_1();
	double calc_u(double e_k);
	void reset();
	void merge(double u_k_1_);

private:
	double dt = 0.0;
	double e_k_1 = 0.0;
	double Kd = 0.0;
	double start = true;

	double u_k_1 = 0.0;
	double tau = 0.0;
};

#endif