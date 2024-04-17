#include "I.h"

I_controller::I_controller() {
	dt = 0.0;
	u_k_1 = 0.0;
	Ki = 0.0;
}

void I_controller::init(double dt_, double Ki_, double u_k_1_, double I_max_) {
	dt = dt_;
	Ki = Ki_;
	u_k_1 = u_k_1_;
	I_max = I_max_;
}

void I_controller::set_param(double dt_, double Ki_, double u_k_1_, double I_max_) {
	dt = dt_;
	Ki = Ki_;
	u_k_1 = u_k_1_;
	I_max = I_max_;
}

void I_controller::set_dt(double dt_) {
	dt = dt_;
}

void I_controller::set_Ki(double Ki_) {
	Ki = Ki_;
}

void I_controller::set_u_0(double u_k_1_) {
	u_k_1 = u_k_1_;
}

void I_controller::set_I_max(double I_max_) {
	I_max = I_max_;
}

double I_controller::get_dt() {
	return dt;
}

double I_controller::get_Ki() {
	return Ki;
}

double I_controller::get_u_k_1() {
	return u_k_1;
}

double I_controller::get_I_max() {
	return I_max;
}

double I_controller::calc_u(double e_k) {
	double u_k = u_k_1 + Ki * dt * e_k;
	u_k = math_fun.saturate(u_k, -I_max, I_max);
	u_k_1 = u_k;
	return u_k;
}

void I_controller::reset() {
	u_k_1 = 0.0;
}

void I_controller::merge(double u_k_1_) {
	u_k_1 = u_k_1_;
}