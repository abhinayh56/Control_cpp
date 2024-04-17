#ifndef CONTROL_SYSTEM_P
#define CONTROL_SYSTEM_P

class P_controller {
public:
	P_controller();
	void init(double Kp_);
	void set_param(double Kp_);
	void set_Kp(double Kp_);
	double get_Kp();
	double calc_u(double e_k);
	void reset();

private:
	double Kp = 0.0;
};

#endif