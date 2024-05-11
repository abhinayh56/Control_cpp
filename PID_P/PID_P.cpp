#include "PID_P.h"

PID_P_controller::PID_P_controller(){
    dt = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    fc = 0.0;
    I_max = 0.0;
    u_max = 0.0;
    d_filter = false;
    e_k_1 = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
    u_ff = 0.0;
    start = true;
}

void PID_P_controller::init(double dt_, double Kp_, double Ki_, double Kd_, double I_max_, double u_max_, bool d_filter_=false, double fc_){
    dt = dt_;
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    I_max = I_max_;
    u_max = u_max_;
    d_filter = d_filter_;
    fc = fc_;
    start = true;
    lpf.set_param(fc,dt);
}

void PID_P_controller::set_param(double dt_, double Kp_, double Ki_, double Kd_, double I_max_, double u_max_, bool d_filter_=false, double fc_){
    dt = dt_;
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    I_max = I_max_;
    u_max = u_max_;
    d_filter = d_filter_;
    fc = fc_;
    lpf.set_param(fc,dt);
}

double PID_P_controller::update(double e_k, double u_ff_);

void PID_P_controller::reset(){
    e_k_1 = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
    u_ff = 0.0;
    start = false;
    lpf.reset();
}

void PID_P_controller::merge(double u_k_1_);

void PID_P_controller::set_dt(double dt_){
    dt = dt_;
}

void PID_P_controller::set_Kp(double Kp_){
    Kp = Kp_;
}

void PID_P_controller::set_Ki(double Ki_){
    Ki = Ki_;
}

void PID_P_controller::set_Kd(double Kd_){
    Kd = Kd_;
}

void PID_P_controller::set_I_max(double I_max_){
    I_max = I_max_;
}

void PID_P_controller::set_u_max(double u_max_){
    u_max = u_max_;
}

void PID_P_controller::set_d_filter(bool d_filter_){
    d_filter = d_filter_;
}

void PID_P_controller::set_fc(double fc_){
    fc = fc_;
}

void PID_P_controller::set_ff(double u_ff_){
    u_ff = u_ff_;
}

void PID_P_controller::set_u_0(double u_k_1_);

double PID_P_controller::get_dt(){
    return dt;
}

double PID_P_controller::get_Kp(){
    return Kp;
}

double PID_P_controller::get_Ki(){
    return Ki;
}

double PID_P_controller::get_Kd(){
    return Kd;
}

double PID_P_controller::get_I_max(){
    return I_max;
}

double PID_P_controller::get_u_max(){
    return u_max;
}

bool PID_P_controller::get_d_filter(){
    return d_filter;
}

double PID_P_controller::get_fc(){
    return fc;
}

double PID_P_controller::get_ff(){
    return u_ff;
}

double PID_P_controller::get_u_0();

double PID_P_controller::get_P(){
    return P;
}

double PID_P_controller::get_I(){
    return I;
}

double PID_P_controller::get_D(){
    return D;
}

double PID_P_controller::get_u(){
    return u;
}