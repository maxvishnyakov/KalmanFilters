#ifndef CONSTVELOCITYMODEL_H
#define CONSTVELOCITYMODEL_H

#include <armadillo>
#include <vector>

struct Point{
    float x;
    float y;
};


struct ConstVelocityModel
{
private:
    enum State{
        StateX,
        StateY,
        StateVx,
        StateVy
    };

public:
    typedef arma::fvec::fixed<4>        state_vec;
    typedef arma::fvec::fixed<2>        measurement_vec;
    typedef arma::fmat::fixed<4, 4>     covariance_mat;
    typedef std::vector<Point>          stations_pos;
    typedef arma::fmat::fixed<2, 4>     m_jakobian_mat;

    ConstVelocityModel(const stations_pos & pos);
    state_vec state;
    covariance_mat P;
    static covariance_mat Q;
    arma::fmat::fixed<2, 2> R;
    const stations_pos & st_pos;

    measurement_vec get_measurements(const state_vec & state);
    m_jakobian_mat measurement_jakobian(const state_vec & state);
    state_vec f_func(const state_vec & state, float dt);
    covariance_mat f_jakobian(const state_vec & state, float dt);
    void residual_check(ConstVelocityModel::measurement_vec &residual);




};

#endif // CONSTVELOCITYMODEL_H
