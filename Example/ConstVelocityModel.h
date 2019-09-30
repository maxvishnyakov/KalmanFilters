#ifndef CONSTVELOCITYMODEL_H
#define CONSTVELOCITYMODEL_H

#include <armadillo>
#include <vector>
#include "MathExpressions.h"

struct Point{
    float x;
    float y;
};


class ConstVelocityModel
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
    typedef std::vector<Point>          stations_pos;

    ConstVelocityModel(const stations_pos & pos);
    //Mandatory model member in KalmanFilter
    state_vec state;
    state_covariance P;

    //Doesn't matter static or no. Filter use model.Q or model.R
    static state_covariance Q;
    arma::fmat::fixed<2, 2> R;
    //TODO: add template spec for model.Q(), model.R(), model.state() and model.P()

    //Member for additional parameter in get_measurement function
    const stations_pos & st_pos;


    //Mandatory functions for KalmanFilter
    measurement_vec get_measurements(const state_vec & state);
    state_vec f_func(const state_vec & state, float dt);

    //Mandatory functions for ExtendedKalmanFilter
    m_jakobian_mat measurement_jakobian(const state_vec & state);
    state_covariance f_jakobian(const state_vec & state, float dt);

    //Optionally function for handle exceptions in residual
    //You may not define this function
    void residual_check(ConstVelocityModel::measurement_vec &residual);
};

#endif // CONSTVELOCITYMODEL_H
