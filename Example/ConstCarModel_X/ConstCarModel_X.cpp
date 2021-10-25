#include "ConstCarModel_X.h"
#include <cmath>

ConstCarModel_X::ConstCarModel_X()
    : H{1, 0}
    , R{50}
{
}

ConstCarModel_X::state_covariance ConstCarModel_X::Q
{
    { 0.1f,     0},
    {    0,  0.001f},
};

ConstCarModel_X::measurement_vec
ConstCarModel_X::get_measurements(const ConstCarModel_X::state_vec &state)
{

    return {state.at(StateX)};
}

ConstCarModel_X::state_vec
ConstCarModel_X::f_func(const ConstCarModel_X::state_vec &state, float dt)
{
    state_vec st;
    st.at(StateX) = state.at(StateX) + state.at(StateVx) * dt;
    st.at(StateVx) = state.at(StateVx);
    return st;
}

ConstCarModel_X::state_covariance
ConstCarModel_X::m_mat(const ConstCarModel_X::state_vec &state, float dt)
{
    return state_covariance{
        { 1,     dt},
        {   0,  1},
    };
}



/*
void ConstCarModel_X::residual_check(ConstCarModel_X::measurement_vec &residual)
{
    for(auto & i : residual){
        if(i > static_cast<float>(1.5 * M_PI)){
            i -= static_cast<float>(2 * M_PI);
        }
        else if (i < static_cast<float>(-1.5 * M_PI)) {
            i += static_cast<float>(2 * M_PI);
        }
    }
}
*/
