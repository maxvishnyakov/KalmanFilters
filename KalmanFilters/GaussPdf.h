#ifndef GAUSSPDF_H
#define GAUSSPDF_H

#include <cmath>
#include "CheckResidual.h"

namespace KalmanFilters {

template<typename Model,
         typename measurement,
         typename covariance>
float gauss_pdf(const measurement & meas,
                const measurement & pred_meas,
                const covariance & S)
{
    measurement residual = meas - pred_meas;
    residual_check<Model>(residual);
    float e = 0.5f * sum((residual % solve_(S, residual)));
    //FIXME: static_cast<float> - явная бредятина (наверное)
    e += 0.5f * (size(meas).n_rows * static_cast<float>(log(2.0 * M_PI))) +
         0.5f + log(det(S));
    return std::exp(-e);
}

template<typename measurement,
         typename covariance>
float gauss_pdf(const measurement & meas,
                const measurement & pred_meas,
                const covariance & S)
{
    measurement residual = meas - pred_meas;
    float e = 0.5f * sum((residual % solve_(S, residual)));
    //FIXME: static_cast<float> - явная бредятина (наверное)
    e += 0.5f * (size(meas).n_rows * static_cast<float>(log(2.0 * M_PI))) +
         0.5f + log(det(S));
    return std::exp(-e);
}

} //namespace KalmanFilters

#endif // GAUSSPDF_H
