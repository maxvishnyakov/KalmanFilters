#ifndef GAUSSPDF_H
#define GAUSSPDF_H

#include <cmath>
#include "CheckResidual.h"

namespace KalmanFilters {

template<typename Model,
         typename measurement,
         typename covariance>
float gauss_pdf(const measurement & value,
                const measurement & mean,
                const covariance & S)
{
    measurement residual = value - mean;
    residual_check<Model>(residual);
    float e = 0.5f * sum_((residual % solve_(S, residual)));
    e += 0.5f * (n_rows(size_(value)) * static_cast<float>(std::log(2.0 * M_PI))) +
         0.5f + std::log(determinant(S));
    return std::exp(-e);
}

template<typename measurement,
         typename covariance>
float gauss_pdf(const measurement & value,
                const measurement & mean,
                const covariance & S)
{
    measurement residual = value - mean;
    float e = 0.5f * sum_((residual % solve_(S, residual)));
    e += 0.5f * (n_rows(size_(value)) * static_cast<float>(std::log(2.0 * M_PI))) +
         0.5f + std::log(determinant(S));
    return std::exp(-e);
}

} //namespace KalmanFilters

#endif // GAUSSPDF_H
