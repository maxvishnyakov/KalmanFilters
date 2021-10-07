#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <tuple>
#include <cmath>

using namespace std;

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
    ExtendedKalmanFilter()
    {}

    float validate_measurements(Model & model,
                                const measurement_vec & measurements,
                                float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(model, dt);

        measurement_vec     pred_meas   = model.get_measurements(pred_state);
        m_jakobian_mat      H           = model.measurement_jakobian(pred_state);
        meas_covariance     S           = H * pred_P * transpose(H) + model.R;
        measurement_vec     residual    = measurements - pred_meas;

    }

    float update(Model & model,
                 const measurement_vec & measurements,
                 float dt)
    {
        state_vec           pred_state;
        state_covariance    pred_P;
        std::tie(pred_state, pred_P)    = predict(model, dt);

        measurement_vec     pred_meas   = model.get_measurements(pred_state);
        m_jakobian_mat      H           = model.measurement_jakobian(pred_state);
        meas_covariance     S           = H * pred_P * transpose(H) + model.R;
        cout<<H<<endl;
        cout<<S<<endl;
        filter_gain_mat     W           = pred_P * transpose(H) * inverse(S);
        measurement_vec     residual    = measurements - pred_meas;

        model.state    = pred_state + (W * residual);
        model.P        = pred_P - W * S * transpose(W);

    }

    void update(Model & model,
                float dt)
    {
        std::tie(model.state, model.P) = predict(dt);
    }


private:

    std::tuple<state_vec, state_covariance>
    predict(Model & model,
            float dt)
    {
        state_vec           state   = model.f_func(model.state, dt);
        state_covariance    A       = model.f_jakobian(model.state, dt);
        state_covariance    P       = A * model.P * transpose(A) + model.Q;
        return std::make_tuple(state, P);
    }
};

}

#endif // EXTENDEDKALMANFILTER_H

/*
std::tuple<state_vec, state_covariance>
predict(Model & model,
        float dt)
{
    state_vec           state   = model.f_func(model.state, dt);
    state_covariance    A       = model.f_jakobian(model.state, dt);
    state_covariance    P       = A * model.P * transpose(A) + model.Q;
    return std::make_tuple(state, P);
}
};
*/
