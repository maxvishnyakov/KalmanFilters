#include "ConstVelocityModel.h"
#include <cmath>

ConstVelocityModel::covariance_mat ConstVelocityModel::Q(arma::fill::zeros);

ConstVelocityModel::ConstVelocityModel(const stations_pos &pos)
    : st_pos(pos)
{

}

ConstVelocityModel::measurement_vec
ConstVelocityModel::get_measurements(const ConstVelocityModel::state_vec &state)
{
    measurement_vec ret;
    const float & x = state.at(StateX);
    const float & y = state.at(StateY);
    auto size = ret.size();
    for(size_t i = 0; i < size; ++i){
        ret.at(i) = static_cast<float>(
                        atan2(static_cast<double>(y - st_pos[i].y),
                              static_cast<double>(x - st_pos[i].x)));
        if(ret.at(i) < 0){
            ret.at(i) += static_cast<float>(2 * M_PI);
        }
    }
    return ret;
}

ConstVelocityModel::m_jakobian_mat
ConstVelocityModel::measurement_jakobian(const ConstVelocityModel::state_vec &state)
{
    m_jakobian_mat ret(arma::fill::zeros);
    for(size_t i = 0; i < 2; ++i){
        auto & st = st_pos[i];
        auto dx = state.at(StateX) - static_cast<float>(st.x);
        auto dy = state.at(StateY) - static_cast<float>(st.y);
        auto denominator = std::pow(dx, 2.0f) + std::pow(dy, 2.0f);
        ret.at(i, StateX) = -dy / denominator;
        ret.at(i, StateY) = dx / denominator;
    }
    return ret;
}

ConstVelocityModel::state_vec
ConstVelocityModel::f_func(const ConstVelocityModel::state_vec &state, float dt)
{
    state_vec st;
    st.at(StateX) = state.at(StateX) + state.at(StateVx) * dt;
    st.at(StateY) = state.at(StateY) + state.at(StateVy) * dt;
    st.at(StateVx) = state.at(StateVx);
    st.at(StateVy) = state.at(StateVy);
    return st;
}

ConstVelocityModel::covariance_mat
ConstVelocityModel::f_jakobian(const ConstVelocityModel::state_vec &state, float dt)
{
    return covariance_mat{
        {1.0,   0,  dt,   0},
        {  0, 1.0,   0,  dt},
        {  0,   0, 1.0,   0},
        {  0,   0,   0, 1.0}
    };
}

void ConstVelocityModel::residual_check(ConstVelocityModel::measurement_vec &residual)
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
