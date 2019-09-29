#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <CheckResidual.h>
#include <tuple>
#include <armadillo>

template <typename Model>
class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(Model & model)
        : model_(model){

    }

    template<typename Measurement>
    float validate_measurement(const Measurement & measurement,
                               float dt)
    {
        //TODO: тут предстоит подумать посерьезнее
//        auto src_id = measurement.source_id();
//        auto pred = predict(model_.state, model_.P, dt);
//        auto predict_meas = model_.get_measurements(pred.state);
//        auto V = model_.meas_jakobian(pred.state);
//        auto nev_temp = meas.azimuth() - predict_meas.at(src_id);
//        //TODO: проверка невязки
//        if(nev_temp > static_cast<float>(1.5 * M_PI)){
//            nev_temp -= static_cast<float>(2 * M_PI);
//        }
//        else if (nev_temp < static_cast<float>(-1.5 * M_PI)) {
//            nev_temp += static_cast<float>(2 * M_PI);
//        }
//        auto nev = pow(nev_temp, 2.0f);
//        auto S = V * pred.P * trans(V) + R;
//        //FIXME это вообще законно?
//        return std::abs(nev / S.at(src_id, src_id));
    }

    float update(const typename Model::measurement_vec & measurements,
                 float dt)
    {
        decltype (Model::state) pred_state;
        decltype (Model::P) pred_P;
        std::tie(pred_state, pred_P) = predict(dt);
        typename Model::measurement_vec pred_meas = model_.get_measurements(pred_state);
        auto H = model_.measurement_jakobian(pred_state);
        auto S = H * pred_P * arma::trans(H) + model_.R;
        auto W = pred_P * arma::trans(H) * arma::inv(S);
        typename Model::measurement_vec nev = measurements - pred_meas;
        residual_check<Model>(nev);
        model_.state = pred_state + W * nev;
        model_.P = pred_P - W * S * arma::trans(W);
        return gauss_pdf(measurements, pred_meas, S);
    }

    void update(float dt)
    {
        std::tie(model_.state, model_.P) = predict(dt);
    }


private:
    Model & model_;

    std::tuple<decltype (Model::state),
               decltype (Model::P)> predict(float dt)
    {
        decltype (Model::state) state = model_.f_func(model_.state, dt);
        decltype (Model::P) A = model_.f_jakobian(model_.state, dt);
        decltype (Model::P) P = A * P * trans(A) + model_.Q;
        return std::make_tuple(state, P);
    }

    float gauss_pdf(const typename Model::measurement_vec & meas,
                    const typename Model::measurement_vec & pred_meas,
                    const arma::fmat & S)
    {
        typename Model::measurement_vec dx = meas - pred_meas;
        residual_check<Model>(dx);
        float e = 0.5f * arma::sum((dx % solve(S, dx)));
        e += 0.5f * (size(meas).n_rows * static_cast<float>(std::log(2.0 * M_PI))) +
             0.5f + log(det(S));
        return std::exp(-e);
    }
};

#endif // EXTENDEDKALMANFILTER_H
