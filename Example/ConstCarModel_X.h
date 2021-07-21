#ifndef CONSTCARMODEL_X_H
#define CONSTCARMODEL_X_H

#include <armadillo>
#include <vector>
#include "MathExpressions.h"

class ConstCarModel_X
{
private:
    enum State{
        StateX,
        StateVx,
    };

public:
    //Mandatory type definition for KalmanFilter
    typedef arma::fvec::fixed<1>        measurement_vec;
    typedef arma::fmat::fixed<1, 1>     meas_covariance;
    typedef arma::fmat::fixed<2, 1>     filter_gain_mat;

    //Optional type definition (filter doesn't use it)
    typedef arma::fmat::fixed<2,2>      state_covariance;
    typedef arma::fvec::fixed<2>        state_vec;

    ConstCarModel_X();
    //Mandatory model member in KalmanFilter
    state_vec state;
    state_covariance P;

    //Doesn't matter static or no. Filter use model.Q or model.R
    static state_covariance Q;
    arma::fmat::fixed<1, 1> R;
    arma::fmat::fixed<1, 2> H;
    //TODO: add template spec for model.Q(), model.R(), model.state() and model.P()

    //Mandatory functions for KalmanFilter
    measurement_vec get_measurements(const state_vec & state);
    state_vec f_func(const state_vec & state, float dt);

    state_covariance m_mat(const state_vec & state, float dt);

    //Optionally function for handle exceptions in residual
    //You may not define this function
    void residual_check(measurement_vec &residual);
};

#endif // CONSTCARMODEL_X_H
