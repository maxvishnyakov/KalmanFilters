#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <CheckResidual.h>
#include <tuple>
#include <cmath>
#include <armadillo>

template <typename Model>
class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(Model & model)
        : model_(model){

    }

    float validate_measurements(const typename Model::measurement_vec & measurements,
                                float dt)
    {
        decltype (Model::state) pred_state;
        decltype (Model::P)     pred_P;
        std::tie(pred_state, pred_P) = predict(dt);
        typename Model::measurement_vec pred_meas = model_.get_measurements(pred_state);
        typename Model::m_jakobian_mat H = model_.measurement_jakobian(pred_state);
        typename Model::meas_covariance S = H * pred_P * arma::trans(H) + model_.R;
        typename Model::measurement_vec residual = measurements - pred_meas;
        residual_check<Model>(residual);
        return as_scalar((arma::trans(residual) * arma::inv(S) * residual));
    }

    float update(const typename Model::measurement_vec & measurements,
                 float dt)
    {
        decltype (Model::state) pred_state;
        decltype (Model::P)     pred_P;
        std::tie(pred_state, pred_P) = predict(dt);
        typename Model::measurement_vec pred_meas = model_.get_measurements(pred_state);
        typename Model::m_jakobian_mat H = model_.measurement_jakobian(pred_state);
        typename Model::meas_covariance S = H * pred_P * arma::trans(H) + model_.R;
        typename Model::filter_gain_mat W = pred_P * arma::trans(H) * arma::inv(S);
        typename Model::measurement_vec residual = measurements - pred_meas;
        residual_check<Model>(residual);
        model_.state = pred_state + (W * residual);
        model_.P = pred_P - W * S * arma::trans(W);
        return gauss_pdf(measurements, pred_meas, S);
    }

    void update(float dt)
    {
        std::tie(model_.state, model_.P) = predict(dt);
    }


private:
    Model & model_;

    std::tuple<decltype (Model::state), decltype (Model::P)>
    predict(float dt)
    {
        decltype (Model::state) state = model_.f_func(model_.state, dt);
        decltype (Model::P)     A = model_.f_jakobian(model_.state, dt);
        decltype (Model::P)     P = A * model_.P * arma::trans(A) + model_.Q;
        return std::make_tuple(state, P);
    }

    template<typename MeasCovariance>
    float gauss_pdf(const typename Model::measurement_vec & meas,
                    const typename Model::measurement_vec & pred_meas,
                    const MeasCovariance & S)
    {
        typename Model::measurement_vec dx = meas - pred_meas;
        residual_check<Model>(dx);
        float e = 0.5f * sum((dx % solve_(S, dx)));
        e += 0.5f * (size(meas).n_rows * static_cast<float>(log(2.0 * M_PI))) +
             0.5f + log(det(S));
        return std::exp(-e);
    }
};

#endif // EXTENDEDKALMANFILTER_H
