#include "ConstCarModel_A_R.h"
#include <cmath>

ConstCarModel_A_R::ConstCarModel_A_R()
{
}

ConstCarModel_A_R::state_covariance ConstCarModel_A_R::Q
{
    { 0.0001f,       0,        0,         0},
    {    0,    0.0001f,        0,         0},
    {    0,         0,    0.0001f,         0},
    {    0,         0,       0,      0.0001f},
};

ConstCarModel_A_R::meas_covariance ConstCarModel_A_R::R
{
     {(0.5*M_PI/180),    0},
     {             0,   50},
};

ConstCarModel_A_R::measurement_vec
ConstCarModel_A_R::get_measurements(const ConstCarModel_A_R::state_vec &state)
{
    measurement_vec ret;
    const float & x = state.at(StateX);
    const float & y = state.at(StateY);

    float den_r = pow(x,2) + pow(y,2);
    float a_meas = atan2(y,x);
    float r_meas = sqrt(den_r);

    ret.at(0) = a_meas;
    ret.at(1) = r_meas;

    return ret;
}

ConstCarModel_A_R::m_jakobian_mat
ConstCarModel_A_R::measurement_jakobian(const ConstCarModel_A_R::state_vec &state)
{
    m_jakobian_mat ret(arma::fill::zeros);
    auto x = state.at(0);
    auto y = state.at(1);

    auto denominator_da_dx = (pow(x,2) * (1 + pow(y,2)/pow(x,2)));
    auto denominator_da_dy = x * (1 + pow(y,2)/pow(x,2));
    auto da_dx = -(y/denominator_da_dx);
    auto da_dy = 1 / denominator_da_dy;

    auto denominator_dr = pow(x,2) + pow(y,2);
    auto dr_dx = x / sqrt(denominator_dr);
    auto dr_dy = y / sqrt(denominator_dr);

    ret(0,0) = da_dx;
    ret(0,1) = da_dy;
    ret(1,0) = dr_dx;
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

