#ifndef CHECKRESIDUAL_H
#define CHECKRESIDUAL_H

#include <type_traits>

namespace KalmanFilters {

//struct for checking the residual check function in model class
template <typename T, typename Input>
struct has_residual_check{
    typedef char yes[1];
    typedef char no[2];

    template<typename Type,Type Ptr>
                struct MemberHelperClass;

    template <typename C>
    static yes & test(MemberHelperClass<void (*)(Input &),&C::residual_check> *);

    template <typename> static no & test(...);

    static const bool value =
            sizeof (test<T>(0)) == sizeof (yes);
};

//if model hasn't residual check - nothing to do
template<typename> void residual_check(...){}


//if has - call this function
template <typename T, typename Input,
          typename = typename std::enable_if<has_residual_check<T, Input>::value>::type>
void residual_check(Input & arg)
{
    T::residual_check(arg);
}

} //namespace KalmanFilters

#endif // CHECKRESIDUAL_H
