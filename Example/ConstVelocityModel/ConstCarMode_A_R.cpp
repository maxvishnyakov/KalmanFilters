#include "ConstCarMode_A_R.h"
#include <cmath>

ConstCarModel_A_R::ConstCarModel_A_R()
{
}

ConstCarModel_A_R::state_covariance ConstCarModel_A_R::Q
{
    { 0.1f,       0,       0,       0},
    {    0,  0.001f,       0,       0},
    {    0,       0,  0.001f,       0},
    {    0,       0,       0,  0.001f},
};

ConstCarModel_A_R::meas_covariance ConstCarModel_A_R::R
{
     {(0.5*M_PI/180),    0},
     {             0,   50},
};

ConstCarModel_A_R::measurement_vec
ConstCarModel_A_R::get_measurements(const ConstCarModel_A_R::state_vec &state)
{
    return {state.at(StateX), state.at(StateY)};

    /*
    measurement_vec ret;
    const float & x = state.at(StateX);
    const float & y = state.at(StateY);

    float a_meas = atan2(y,x);
    float r_meas = sin(a_meas) / x;

    ret.at(0) = a_meas;
    ret.at(1) = r_meas;

    return ret;
    */
}

ConstCarModel_A_R::m_jakobian_mat
ConstCarModel_A_R::measurement_jakobian(const ConstCarModel_A_R::state_vec &state)
{
    m_jakobian_mat ret(arma::fill::zeros);
    auto x = state.at(0);
    auto y = state.at(1);

    auto da_dx = -(y/(1 + pow(y,2)));
    auto da_dy = (x/1+ pow((y/x),2));

    auto denominator_dr_dx = pow(x,2) + pow(y,2);
    auto dr_dx = x / pow(denominator_dr_dx, 1/2);
    auto dr_dy = y / pow(denominator_dr_dx, 1/2);

    ret(0,0) = da_dx;
    ret(1,0) = da_dy;
    ret(0,1) = dr_dx;
    ret(1,1) = dr_dy;

    return ret;
}

ConstCarModel_A_R::state_vec
ConstCarModel_A_R::f_func(const ConstCarModel_A_R::state_vec &state, float dt)
{
    state_vec st;
    st.at(StateX) = state.at(StateX) + state.at(StateVx) * dt;
    st.at(StateY) = state.at(StateY) + state.at(StateVy) * dt;
    st.at(StateVx) = state.at(StateVx);
    st.at(StateVy) = state.at(StateVy);
    return st;
}

ConstCarModel_A_R::state_covariance
ConstCarModel_A_R::f_jakobian(const ConstCarModel_A_R::state_vec &state, float dt)
{
        return state_covariance{
            {1.0,   0,  dt,   0},
            {  0, 1.0,   0,  dt},
            {  0,   0, 1.0,   0},
            {  0,   0,   0, 1.0}
    };
}
