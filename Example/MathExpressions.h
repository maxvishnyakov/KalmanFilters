#ifndef MATHEXPRESSIONS_H
#define MATHEXPRESSIONS_H

#include <armadillo>

template <typename Matrix>
auto transpose(const Matrix & matrix) -> decltype (arma::trans(matrix))
{
    return arma::trans(matrix);
}

template <typename Matrix>
auto inverse(const Matrix & matrix) -> decltype (arma::inv(matrix))
{
    return arma::inv(matrix);
}

template <typename T>
auto as_scalar(const T & t) -> decltype (arma::as_scalar(t))
{
    return arma::as_scalar(t);
}

template <typename M, typename C>
auto solve_(const M & m, const C & c) -> decltype (arma::solve(m, c))
{
    return arma::solve(m, c);
}

#endif // MATHEXPRESSIONS_H
