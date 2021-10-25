#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <tuple>
#include <cmath>

#include "CheckResidual.h"
#include "GaussPdf.h"

namespace KalmanFilters
{

template <typename Model>
class KalmanFilter
{
    typedef     decltype (Model::state)             state_vec;
    typedef     decltype (Model::P)                 state_covariance;
    typedef     typename Model::measurement_vec     measurement_vec;
    typedef     typename Model::meas_covariance     meas_covariance;
    typedef     typename Model::filter_gain_mat     filter_gain_mat;
    //typedef     typename Model::m_mat               m_mat;

public:
    KalmanFilter()
    {
    };

    float validate_measurements(Model & model,
                                const measurement_vec & measurements,
                                float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(model, dt);

        measurement_vec     pred_meas   = model.get_measurements(pred_state);
        meas_covariance     S           = model.H * pred_P * transpose(model.H) + model.R;
        measurement_vec     residual    = measurements - pred_meas;

        return to_scalar((transpose(residual) * inverse(S) * residual));
    }

    void update(Model & model,
                 const measurement_vec & measurements,
                 float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(model, dt);

        measurement_vec     pred_meas   = model.get_measurements(pred_state);
        meas_covariance     S           = model.H * pred_P * transpose(model.H) + model.R;
        filter_gain_mat     W           = pred_P * transpose(model.H) * inverse(S);
        measurement_vec     residual    = measurements - pred_meas;

        model.state    = pred_state + (W * residual);
        model.P        = pred_P - (W * S * transpose(W));
    }

    void update(Model & model,
                float dt)
    {
        std::tie(model.state, model.P) = predict(model, dt);
    }


private:

    std::tuple<state_vec, state_covariance>
    predict(Model & model,
            float dt)
    {
        state_vec           state   = model.f_func(model.state, dt);
        state_covariance    A       = model.m_mat(model.state, dt);
        state_covariance    P       = A * model.P * transpose(A) + model.Q;
        return std::make_tuple(state, P);
    }
};
}


#endif // KALMANFILTER_H
