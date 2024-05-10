#ifndef CONTROL_SYSTEM_PID
#define CONTROL_SYSTEM_PID

#include <stdint.h>
#include "Math_functions.h"

class PID_LPF_1_FF_controller {
	public:
		PID_LPF_1_FF_controller();
		void init(double dt_, double Kp_, double Ki_, double Kd_, double fc_, double u_k_1_, double u_k_2_, double u_max_);
		void set_param(double dt_, double Kp_, double Ki_, double Kd_, double fc_, double u_k_1_, double u_k_2_, double u_max_);
		double update(double e_k, double u_ff_);
		void reset();
		void merge(double u_k_1_);
		void set_dt(double dt_);
		void set_Kp(double Kp_);
		void set_Ki(double Ki_);
		void set_Kd(double Kd_);
		void set_fc(double fc_);
		void set_ff(double u_ff_);
		void set_u_0(double u_k_1_);
		void set_u_max(double u_max_);
		double get_dt();
		double get_Kp();
		double get_Ki();
		double get_Kd();
		double get_fc();
		double get_ff();
		double get_u_k_1();
		double get_e_k_1();
		double get_e_k_2();
		double get_u_max();

	private:
		Math_functions math_fun;
		double dt = 0.0;
		double u_k_1 = 0.0;
		double u_k_2 = 0.0;
		double e_k_1 = 0.0;
		double e_k_2 = 0.0;
		double Kp = 0.0;
		double Ki = 0.0;
		double Kd = 0.0;
		double tau = 0.0;
		double u_ff = 0.0;
		double u_max = 9999999999;
		uint8_t start = 0;
};

#endif