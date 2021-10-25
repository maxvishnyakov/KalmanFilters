#ifndef CONSTVELOCITYMODEL_H
#define CONSTVELOCITYMODEL_H

#include <armadillo>
#include <vector>

#include "../MathExpressions.h"
class ConstCarModel_A_R
{
private:
    enum State{
        StateX,
        StateY,
        StateVx,
        StateVy
    };

public:
    //Mandatory type definition for KalmanFilter
    typedef arma::fvec::fixed<2>        measurement_vec;
    typedef arma::fmat::fixed<2, 2>     meas_covariance;
    typedef arma::fmat::fixed<4, 2>     filter_gain_mat;

    //Mandatory type definition for ExtendedKalmanFilter
    typedef arma::fmat::fixed<2, 4>     m_jakobian_mat;

    //Optional type definition (filter doesn't use it)
    typedef arma::fmat::fixed<4, 4>     state_covariance;
    typedef arma::fvec::fixed<4>        state_vec;

    ConstCarModel_A_R();

    state_vec state;
    state_covariance P;
    static meas_covariance R;

    static state_covariance Q;

    measurement_vec get_measurements(const state_vec & state);
    state_vec f_func(const state_vec & state, float dt);

    m_jakobian_mat measurement_jakobian(const state_vec & state);
    state_covariance f_jakobian(const state_vec & state, float dt);

    void residual_check(measurement_vec &residual);
};

#endif // CONSTVELOCITYMODEL_H

