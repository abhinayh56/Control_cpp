#include "P.h"

P_controller::P_controller() {
	Kp = 0.0;
}

void P_controller::init(double Kp_) {
	Kp = Kp_;
}

void P_controller::set_param(double Kp_) {
	Kp = Kp_;
}

void P_controller::set_Kp(double Kp_) {
	Kp = Kp_;
}

double P_controller::get_Kp() {
	return Kp;
}

double P_controller::calc_u(double e_k) {
	return Kp * e_k;
}

void P_controller::reset() {
}