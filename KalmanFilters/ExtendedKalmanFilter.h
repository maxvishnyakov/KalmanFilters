#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <tuple>
#include <cmath>

#include "CheckResidual.h"
#include "GaussPdf.h"

namespace KalmanFilters {

template <typename Model>
class ExtendedKalmanFilter
{
    typedef     decltype (Model::state)             state_vec;
    typedef     decltype (Model::P)                 state_covariance;
    typedef     typename Model::measurement_vec     measurement_vec;
    typedef     typename Model::m_jakobian_mat      m_jakobian_mat;
    typedef     typename Model::meas_covariance     meas_covariance;
    typedef     typename Model::filter_gain_mat     filter_gain_mat;


public:
    ExtendedKalmanFilter(Model & model)
        : model_(model){

    }

    float validate_measurements(const measurement_vec & measurements,
                                float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(dt);

        measurement_vec     pred_meas   = model_.get_measurements(pred_state);
        m_jakobian_mat      H           = model_.measurement_jakobian(pred_state);
        meas_covariance     S           = H * pred_P * transpose(H) + model_.R;
        measurement_vec     residual    = measurements - pred_meas;

        residual_check<Model>(residual);

        return to_scalar((transpose(residual) * inverse(S) * residual));
    }

    float update(const measurement_vec & measurements,
                 float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(dt);

        measurement_vec     pred_meas   = model_.get_measurements(pred_state);
        m_jakobian_mat      H           = model_.measurement_jakobian(pred_state);
        meas_covariance     S           = H * pred_P * transpose(H) + model_.R;
        filter_gain_mat     W           = pred_P * transpose(H) * inverse(S);
        measurement_vec     residual    = measurements - pred_meas;

        residual_check<Model>(residual);

        model_.state    = pred_state + (W * residual);
        model_.P        = pred_P - W * S * transpose(W);
        //FIXME: need to make gauss_pdf function optional
        return gauss_pdf<Model>(measurements, pred_meas, S);
    }

    void update(float dt)
    {
        std::tie(model_.state, model_.P) = predict(dt);
    }


private:
    Model & model_;

    std::tuple<state_vec, state_covariance>
    predict(float dt)
    {
        state_vec           state   = model_.f_func(model_.state, dt);
        state_covariance    A       = model_.f_jakobian(model_.state, dt);
        state_covariance    P       = A * model_.P * transpose(A) + model_.Q;
        return std::make_tuple(state, P);
    }
};

} //namespace KalmanFilters

#endif // EXTENDEDKALMANFILTER_H
