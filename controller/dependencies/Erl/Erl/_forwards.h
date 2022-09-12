/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_FORWARDS_H
#define ERL_FORWARDS_H

#include <Erl/_platform/_platform.h>
#include <Eigen/Core>
#include <cassert>
#include <type_traits>

#define ERL_PI	3.14159265358979323846264338327950288419716939937510

#ifndef ERL_TRUE
    #define ERL_TRUE  1
#endif
#ifndef ERL_FALSE
    #define ERL_FALSE 0
#endif

#ifdef ERL_ASSERT_OFF
    #ifndef ERL_ASSERT_ACTIVE
    #define ERL_ASSERT_ACTIVE ERL_FALSE
    #endif
#else
    #ifndef ERL_ASSERT_ACTIVE
    #define ERL_ASSERT_ACTIVE ERL_TRUE
    #endif
#endif

#if defined(ERL_ASSERT_ACTIVE) && (ERL_ASSERT_ACTIVE==ERL_FALSE)
    #ifndef ERL_ASSERT
        #define ERL_ASSERT(assert_expression)
    #endif
    #ifndef ERL_ASSERT_CONSTEXPR
        #define ERL_ASSERT_CONSTEXPR(assert_expression,call) call
    #endif
#else
   #ifndef ERL_ASSERT
        #define ERL_ASSERT(assert_expression) assert(assert_expression)
   #endif
   #ifndef ERL_ASSERT_CONSTEXPR
       #define ERL_ASSERT_CONSTEXPR(assert_expression,call) (constexpr_assert(assert_expression) , call)
   #endif
#endif

namespace details
{
    ERL_RMV_UNPARA_SINGLE
    inline void assert_helper( bool _assert_expression) {  assert(_assert_expression); }
}

#if ERL_CONSTEXPR_ACTIVE==ERL_TRUE
inline ERL_USE_CONSTEXPR bool constexpr_assert( bool _assert_expression)
{  return _assert_expression ? true : ( details::assert_helper(_assert_expression) , false ); }
#else
inline bool constexpr_assert( bool _assert_expression)
{  return _assert_expression ? true : ( details::assert_helper(_assert_expression) , false ); }

#endif


#ifdef ERL_COMMON_ALIGNMENT
    #ifndef ERL_VECTOR3_ALIGNMENT
        #define ERL_VECTOR3_ALIGNMENT         ERL_COMMON_ALIGNMENT
    #endif
    #ifndef ERL_VECTOR6_ALIGNMENT
        #define ERL_VECTOR6_ALIGNMENT         ERL_COMMON_ALIGNMENT
    #endif
    #ifndef ERL_QUATERNION_ALIGNMENT
        #define ERL_QUATERNION_ALIGNMENT      ERL_COMMON_ALIGNMENT
    #endif
    #ifndef ERL_ROTMAT_ALIGNMENT
        #define ERL_ROTMAT_ALIGNMENT          ERL_COMMON_ALIGNMENT
    #endif
    #ifndef ERL_TRANSFORM_ALIGNMENT
        #define ERL_TRANSFORM_ALIGNMENT       ERL_COMMON_ALIGNMENT
    #endif
    #ifndef ERL_DUALQUATERNION_ALIGNMENT
        #define ERL_DUALQUATERNION_ALIGNMENT  ERL_COMMON_ALIGNMENT
    #endif
#endif

#ifndef ERL_VECTOR3_ALIGNMENT
    #define ERL_VECTOR3_ALIGNMENT
    #define ERL_ALIGNAS_VECTOR3
#elif !defined(ERL_ALIGNAS_VECTOR3)
    #define ERL_ALIGNAS_VECTOR3  alignas(::Erl::details::max_alignment<Eigen::Matrix< T, 3, 1 >, ERL_VECTOR3_ALIGNMENT>::value )
#endif
#ifndef ERL_VECTOR6_ALIGNMENT
    #define ERL_VECTOR6_ALIGNMENT
    #define ERL_ALIGNAS_VECTOR6
#elif !defined(ERL_ALIGNAS_VECTOR6)
    #define ERL_ALIGNAS_VECTOR6 alignas(::Erl::details::max_alignment<Eigen::Matrix< T, 6, 1 >, ERL_VECTOR6_ALIGNMENT>::value )
#endif
#ifndef ERL_QUATERNION_ALIGNMENT
    #define ERL_QUATERNION_ALIGNMENT
    #define ERL_ALIGNAS_QUATERNION
#elif !defined(ERL_ALIGNAS_QUATERNION)
    #define ERL_ALIGNAS_QUATERNION  alignas(::Erl::details::max_alignment<Eigen::Matrix< T, 4, 1 >, ERL_QUATERNION_ALIGNMENT>::value)
#endif
#ifndef ERL_ROTMAT_ALIGNMENT
    #define ERL_ROTMAT_ALIGNMENT
    #define ERL_ALIGNAS_ROTMAT
#elif !defined(ERL_ALIGNAS_ROTMAT)
    #define ERL_ALIGNAS_ROTMAT alignas(::Erl::details::max_alignment<Eigen::Matrix< T, 3, 3 >, ERL_ROTMAT_ALIGNMENT>::value )
#endif
#ifndef ERL_TRANSFORM_ALIGNMENT
    #define ERL_TRANSFORM_ALIGNMENT
    #define ERL_ALIGNAS_TRANSFORM
#elif !defined(ERL_ALIGNAS_TRANSFORM)
    #define ERL_ALIGNAS_TRANSFORM alignas(::Erl::details::max_alignment<::Erl::details::transform_determine_alignment< T, ERL_MAJOR_TYPE_VALUE >, ERL_TRANSFORM_ALIGNMENT>::value )
#endif
#ifndef ERL_DUALQUATERNION_ALIGNMENT
    #define ERL_DUALQUATERNION_ALIGNMENT
    #define ERL_ALIGNAS_DUALQUATERNION
#elif !defined(ERL_ALIGNAS_DUALQUATERNION)
    #define ERL_ALIGNAS_DUALQUATERNION alignas(::Erl::details::max_alignment<::Erl::details::dualquaternion_determine_alignment< T >, ERL_DUALQUATERNION_ALIGNMENT>::value )
#endif


namespace Erl
{

enum
{
    RowMajor = Eigen::RowMajor,
    ColMajor = Eigen::ColMajor
};

enum class Axis
{
    X = 0,
    Y = 1,
    Z = 2
};

template<class T>
struct Constants
{
    static ERL_USE_CONSTEXPR T Zero_Tolerance_VeryStrict = 0;
    static ERL_USE_CONSTEXPR T Zero_Tolerance_Strict     = 0;
    static ERL_USE_CONSTEXPR T Zero_Tolerance            = 0;
    static ERL_USE_CONSTEXPR T Zero_Tolerance_Soft       = 0;
};
template<>
struct Constants<long double>
{
    static ERL_USE_CONSTEXPR long double Zero_Tolerance_VeryStrict = 1.e-40l;
    static ERL_USE_CONSTEXPR long double Zero_Tolerance_Strict     = 1.e-20l;
    static ERL_USE_CONSTEXPR long double Zero_Tolerance            = 1.e-10l;
    static ERL_USE_CONSTEXPR long double Zero_Tolerance_Soft       = 1.e-5l;
};
template<>
struct Constants<double>
{
    static ERL_USE_CONSTEXPR double Zero_Tolerance_VeryStrict = 1.e-32;
    static ERL_USE_CONSTEXPR double Zero_Tolerance_Strict     = 1.e-16;
    static ERL_USE_CONSTEXPR double Zero_Tolerance            = 1.e-8;
    static ERL_USE_CONSTEXPR double Zero_Tolerance_Soft       = 1.e-4;
};
template<>
struct Constants<float>
{
    static ERL_USE_CONSTEXPR float Zero_Tolerance_VeryStrict = 1.e-24f;
    static ERL_USE_CONSTEXPR float Zero_Tolerance_Strict     = 1.e-12f;
    static ERL_USE_CONSTEXPR float Zero_Tolerance            = 1.e-6f;
    static ERL_USE_CONSTEXPR float Zero_Tolerance_Soft       = 1.e-3f;
};
template <class T>
ERL_USE_CONSTEXPR T Constants<T>::Zero_Tolerance_Strict ;
template <class T>
ERL_USE_CONSTEXPR T Constants<T>::Zero_Tolerance        ;
template <class T>
ERL_USE_CONSTEXPR T Constants<T>::Zero_Tolerance_Soft   ;

template<class T> class Vector3;
template<class T> class Vector6;
template<class T> class Quaternion;
template<class T, int MajorType = Eigen::ColMajor> class Transform;
template<class T, int MajorType = Eigen::ColMajor> class Rotmat;
template<class T> class MovingFilter;

template<class T> struct DualNumber;
template<class T> class DualQuaternion;

namespace details
{
    template<class _atype, size_t _align>
    struct max_alignment
    {
        static constexpr size_t value =  ((alignof(_atype)>_align)*alignof(_atype)) + ((alignof(_atype)<=_align)*_align);
    };
}

}

#endif //ERL_FORWARDS_H
