#ifndef MATHEXPRESSIONS_H
#define MATHEXPRESSIONS_H

#include <armadillo>

//Файл определения функций с матричными операциями.
//Должен находиться в одной единице трансляции, что и фильтры Калмана
//Т.е. достаточно подключить его в файл с описанием модели
//Теоритически, должен помочь избежать зависимости от armadillo в самой библиотеке фильтров

//TODO: привести к одной форме именования функций

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
auto to_scalar(const T & t) -> decltype (arma::as_scalar(t))
{
    return arma::as_scalar(t);
}

template <typename M, typename C>
auto solve_(const M & m, const C & c) -> decltype (arma::solve(m, c))
{
    return arma::solve(m, c);
}

template <typename M>
auto determinant(const M & m) -> decltype (arma::det(m))
{
    return arma::det(m);
}

template <typename M>
auto sum_(const M & m) -> decltype (arma::sum(m))
{
    return arma::sum(m);
}

template <typename M>
auto size_(const M & m) -> decltype (arma::size(m))
{
    return arma::size(m);
}

template <typename C>
auto n_rows(const C & c) -> decltype (c.n_rows)
{
    return c.n_rows;
}

#endif // MATHEXPRESSIONS_H
