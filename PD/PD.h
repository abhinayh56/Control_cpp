#ifndef CONTROL_SYSTEM_PD
#define CONTROL_SYSTEM_PD

class PD_controller {
public:
	PD_controller();
	void init(double dt_, double Kp_, double Kd_);
	void set_param(double dt_, double Kp_, double Kd_);
	void set_dt(double dt_);
	void set_Kp(double Kp_);
	void set_Kd(double Kd_);
	double get_dt();
	double get_Kp();
	double get_Kd();
	double get_e_k_1();
	double calc_u(double e_k);
	void reset();
	void merge(double u_k_1_);

private:
	double dt = 0.0;
	double e_k_1 = 0.0;
	double Kp = 0.0;
	double Kd = 0.0;
	double start = true;
};

#endif