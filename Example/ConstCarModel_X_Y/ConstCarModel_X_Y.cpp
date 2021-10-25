#include "../ConstCarModel_X_Y.h"
#include <cmath>

ConstCarModel_X_Y::ConstCarModel_X_Y()
{
}

ConstCarModel_X_Y::covariance_h ConstCarModel_X_Y::H
{
    { 1, 0, 0, 0},
    { 0, 1, 0, 0},
};

ConstCarModel_X_Y::meas_covariance ConstCarModel_X_Y::R
{
    { 50,     0},
    {    0,  50},
};

ConstCarModel_X_Y::state_covariance ConstCarModel_X_Y::Q
{
    { 0.1f,       0,       0,       0},
    {    0,  0.001f,       0,       0},
    {    0,       0,  0.001f,       0},
    {    0,       0,       0,  0.001f},
};

ConstCarModel_X_Y::measurement_vec
ConstCarModel_X_Y::get_measurements(const ConstCarModel_X_Y::state_vec &state)
{
    return {state.at(StateX), state.at(StateY)};
}

ConstCarModel_X_Y::state_vec
ConstCarModel_X_Y::f_func(const ConstCarModel_X_Y::state_vec &state, float dt)
{
    state_vec st;
    st.at(StateX) = state.at(StateX) + state.at(StateVx) * dt;
    st.at(StateVx) = state.at(StateVx);
    st.at(StateY) = state.at(StateY) + state.at(StateVx) * dt;
    st.at(StateVy) = state.at(StateVy);
    return st;
}

ConstCarModel_X_Y::state_covariance
ConstCarModel_X_Y::m_mat(const ConstCarModel_X_Y::state_vec &state, float dt)
{
    return state_covariance{
        {1.0,   0,  dt,   0},
        {  0, 1.0,   0,  dt},
        {  0,   0, 1.0,   0},
        {  0,   0,   0, 1.0}
    };
}






