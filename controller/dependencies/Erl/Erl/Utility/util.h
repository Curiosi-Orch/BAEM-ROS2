#ifndef ERL_UTIL_H
#define ERL_UTIL_H

/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#include <Eigen/Dense>
#include "Erl/_forwards.h"
#include <cstdlib>
#include <cmath>
#include <type_traits>
#include <algorithm>
#include <ratio>
#include <tuple>
#include <cassert>

#if defined(__unix__) || defined(__unix)
  #include <unistd.h>
  #if ((defined __QNXNTO__) || (defined _GNU_SOURCE) || (defined __PGI) || ((defined _XOPEN_SOURCE) && (_XOPEN_SOURCE >= 600))) && (defined _POSIX_ADVISORY_INFO) && (_POSIX_ADVISORY_INFO > 0)
    #define ERL_HAS_POSIX_MEMALIGN 1
  #endif
#endif

#ifndef _MSC_VER
#define ERL_REMOVE_GCC_UNUSED_PARAMETER_WARNING _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")
#else
#define ERL_REMOVE_GCC_UNUSED_PARAMETER_WARNING
#endif

#define ERL_NON_INTEGER_TYPE(T) \
    template<typename T> typename std::enable_if<!std::is_integral<T>::value, T>::type

#define ERL_REFUSE_INTEGER_TYPE(func, arg) \
    ERL_REMOVE_GCC_UNUSED_PARAMETER_WARNING \
    template<typename T> \
    typename std::enable_if<std::is_integral<T>::value, T>::type inline func(const T& arg) \
    { static_assert(!std::is_integral<T>::value, "Do not use integer types with this function"); }

namespace Erl
{

ERL_NON_INTEGER_TYPE(T) inline ERL_USE_CONSTEXPR toDegree(const T &radian)
{
    return radian*(T)180/(T)ERL_PI;
}
ERL_REFUSE_INTEGER_TYPE(toDegree, radian)

ERL_NON_INTEGER_TYPE(T) inline ERL_USE_CONSTEXPR toRadian(const T &degree)
{
    return degree*(T)ERL_PI/(T)180;
}
ERL_REFUSE_INTEGER_TYPE(toRadian, degree)

ERL_NON_INTEGER_TYPE(T) inline ERL_USE_CONSTEXPR toRPM(const T &rad_per_sec)
{
    return rad_per_sec*T(60)/(T(2)*T(ERL_PI));
}
ERL_REFUSE_INTEGER_TYPE(toRPM, rad_per_sec)

ERL_NON_INTEGER_TYPE(T) inline ERL_USE_CONSTEXPR toRad_per_Sec(const T &RPM)
{
    return RPM/T(60)*T(2)*T(ERL_PI);
}
ERL_REFUSE_INTEGER_TYPE(toRad_per_Sec, RPM)

template <typename T> inline ERL_USE_CONSTEXPR Vector3<T> toDegree(const Vector3<T> &radian)
{
    return radian*(T)180/(T)ERL_PI;
}

template <typename T> inline ERL_USE_CONSTEXPR Vector3<T> toRadian(const Vector3<T> &degree)
{
    return degree*(T)ERL_PI/(T)180;
}

template<typename T> inline ERL_USE_CONSTEXPR bool equal( const T& v1, const T& v2 )
{
    return std::abs(v1-v2)<=std::nextafter(T(0),T(1));
}

template<typename T> inline ERL_USE_CONSTEXPR bool equal( const T& v1, const T& v2, const T& thres)
{
    return std::abs(v1-v2)<=thres;
}

template<typename V, typename T> inline ERL_USE_CONSTEXPR bool equal( const V& v1, const V& v2, const T& thres)
{
    return std::abs(v1-v2)<=thres;
}

template<typename Tv, typename Ts> inline bool subset( const Tv& v, const Ts& s, const Ts& thres)
{
    return v.unaryExpr([&s,&thres](Ts e){return equal<Ts>(e,s,thres);}).any();
}

template<typename T> inline ERL_USE_CONSTEXPR const T& clamp( const T& v, const T& v_min, const T& v_max)
{
    return ERL_ASSERT_CONSTEXPR( ((v_min<=v_max) && "lower bound larger than upper bound") , (std::min<T>(std::max<T>(v, v_min), v_max)) );
}

template<typename Derived> inline auto clamp_array( const Eigen::ArrayBase<Derived>& v, const Eigen::ArrayBase<Derived>& v_min, const Eigen::ArrayBase<Derived> & v_max)
 -> decltype(v_max.min( v_min.max(v)))
{
    ERL_ASSERT( (v_min<=v_max).all() && "lower bound larger than upper bound");
    return (v_max.min( v_min.max(v))) ;
}

template<typename T> inline  ERL_USE_CONSTEXPR const T& clamp_auto( const T& v, const T& v_Lim1, const T& v_Lim2)
{
    return clamp<T>(v,std::min<T>(v_Lim1,v_Lim2),std::max<T>(v_Lim1,v_Lim2));
}

template<typename T0,typename T1> inline  ERL_USE_CONSTEXPR const T0& min_assign( const T0& a, const T0& b, const T1& _idxa, const T1& _idxb, T1& _idx)
{
    return (b < a) ? ( _idx=_idxb , b ) : (_idx=_idxa , a ) ;
}
template<typename T0,typename T1> inline  ERL_USE_CONSTEXPR const T0& max_assign( const T0& a, const T0& b, const T1& _idxa, const T1& _idxb, T1& _idx)
{
    return (b > a) ? ( _idx=_idxb , b ) : (_idx=_idxa , a ) ;
}

namespace details
{

template <typename T, bool isSigned> struct _signum_helper
{
    static ERL_USE_CONSTEXPR T call(const T& x);
};

template <typename T> struct _signum_helper<T, true>
{
    static ERL_USE_CONSTEXPR T call(const T& x) { return (T(0) < x) - (x < T(0)); }
};

template <typename T> struct _signum_helper<T, false>
{
    static ERL_USE_CONSTEXPR T call(const T& x) { return T(0) < x; }
};

}

template <typename T> inline ERL_USE_CONSTEXPR T signum(const T& x)
{
    return details::_signum_helper<T, std::is_signed<T>::value>::call(x);
}

template <typename T> inline ERL_USE_CONSTEXPR T sign(const T& x)
{
    return T(1)-T(2)*T(std::signbit(x));
}


template<class T,class Ratio = std::ratio<1> > inline ERL_USE_CONSTEXPR T convert(const T& v)
{
    return ((v * Ratio::num)/Ratio::den);
}

template<class T, int num, int den > inline ERL_USE_CONSTEXPR T convert(const T& v)
{
    return ((v * T(num))/T(den));
}

ERL_NON_INTEGER_TYPE(T) inline wrapTo2Pi(const T & _angleRad)
{
    T angle_return = std::fmod(_angleRad, T(2.*ERL_PI) );
    if (_angleRad<T(0)) angle_return+=T(2.*ERL_PI);
    return angle_return;
}
ERL_REFUSE_INTEGER_TYPE(wrapTo2Pi, _angleRad)

ERL_NON_INTEGER_TYPE(T) inline wrapToPi(const T& _angleRad)
{
    return wrapTo2Pi( _angleRad + T(ERL_PI) )-T(ERL_PI);
}
ERL_REFUSE_INTEGER_TYPE(wrapToPi, _angleRad)

ERL_NON_INTEGER_TYPE(T) inline wrapTo360(const T & _angleDeg)
{
    T angle_return = std::fmod(_angleDeg, T(360.) );
    if (_angleDeg<T(0)) angle_return+=T(360.);
    return angle_return;
}
ERL_REFUSE_INTEGER_TYPE(wrapTo360, _angleDeg)

ERL_NON_INTEGER_TYPE(T) inline wrapTo180(const T& _angleDeg)
{
    return wrapTo360( _angleDeg + T(180.) )-T(180.);
}
ERL_REFUSE_INTEGER_TYPE(wrapTo180, _angleDeg)

template<size_t ALIGNMENT>
inline void* aligned_malloc(size_t _size)
{
  void *c_result = nullptr;
  #if defined(__INTEL_COMPILER)
    c_result = _mm_malloc(_size, ALIGNMENT);
  #elif ERL_HAS_POSIX_MEMALIGN
    if(posix_memalign(&c_result, ALIGNMENT, _size)) c_result = nullptr;
  #elif defined(_MSC_VER) && (!defined(_WIN32_WCE))
    c_result = _aligned_malloc(_size, ALIGNMENT);
  #endif

  if(!c_result && _size){throw std::bad_alloc();}
  return c_result;
}

inline void* aligned_malloc(size_t _size, size_t _alignement)
{
  void *c_result = nullptr;
  #if defined(__INTEL_COMPILER)
    c_result = _mm_malloc(_size, _alignement);
  #elif ERL_HAS_POSIX_MEMALIGN
    if(posix_memalign(&c_result, _alignement, _size)) c_result = NULL;
  #elif defined(_MSC_VER) && (!defined(_WIN32_WCE))
    c_result = _aligned_malloc(_size, _alignement);
  #endif

  if(!c_result && _size){throw std::bad_alloc();}
  return c_result;
}

inline void aligned_free(void *_ptr)
{
  #if defined(__INTEL_COMPILER)
    _mm_free(_ptr);
  #elif ERL_HAS_POSIX_MEMALIGN
    std::free(_ptr);
  #elif defined(_MSC_VER) && (!defined(_WIN32_WCE))
    _aligned_free(_ptr);
  #endif
}

template<class T, typename... Args>
inline T* alloc_init_array(size_t _size,const Args&... args)
{
    T * c_ArrayMem = reinterpret_cast<T*>(malloc(sizeof(T)*_size));
    for(size_t i(0);i<_size;i++){ new (&c_ArrayMem[i]) T(args...);}
    return c_ArrayMem;
}

template<class T, typename... Args>
inline T* alloc_init_array(size_t _size,Args&... args)
{
    T * c_ArrayMem = reinterpret_cast<T*>(malloc(sizeof(T)*_size));
    for(size_t i(0);i<_size;i++){ new (&c_ArrayMem[i]) T(args...);}
    return c_ArrayMem;
}

template<class T>
inline void free_init_array(T*& _array,size_t _size)
{
    for(size_t i(0);i<_size;i++){  _array[i].~T();}
    free(reinterpret_cast<void*>(_array));
    _array = nullptr;
}

template<class T, typename... Args>
inline T* alloc_aligned_init_array(size_t _size,size_t _alignement,const Args&... args)
{
    T * c_ArrayMem = reinterpret_cast<T*>(aligned_malloc(sizeof(T)*_size,_alignement));
    for(size_t i(0);i<_size;i++){ new (&c_ArrayMem[i]) T(args...);}
    return c_ArrayMem;
}

template<class T, typename... Args>
inline T* alloc_aligned_init_array(size_t _size,size_t _alignement, Args&... args)
{
    T * c_ArrayMem = reinterpret_cast<T*>(aligned_malloc(sizeof(T)*_size,_alignement));
    for(size_t i(0);i<_size;i++){ new (&c_ArrayMem[i]) T(args...);}
    return c_ArrayMem;
}

template<class T>
inline T* alloc_aligned_init_array_noparameter(size_t _size,size_t _alignement)
{
    T * c_ArrayMem = reinterpret_cast<T*>(aligned_malloc(sizeof(T)*_size,_alignement));
    for(size_t i(0);i<_size;i++){ new (&c_ArrayMem[i]) T();}
    return c_ArrayMem;
}

template<class T>
inline void free_aligned_init_array(T*& _array,size_t _size)
{
    for(size_t i(0);i<_size;i++){_array[i].~T();}
    aligned_free(reinterpret_cast<void*>(_array));
    _array = nullptr;
}

template<size_t _Align>
inline size_t align_padded_size(size_t _byte_size_non_padded)
{
     return ( (_byte_size_non_padded) +(_Align-1)) & ~(_Align-1);
}

inline size_t align_padded_size(size_t _byte_size_non_padded, size_t _alignement)
{
     return ( (_byte_size_non_padded) +(_alignement-1)) & ~(_alignement-1);
}

/// Template parameter extraction. Incompatible with non-type parameters.

template<typename T, int N = 0>
struct extract_value_type { typedef T value_type; };

template<template<typename... Args> class X, typename... Args, int N>
struct extract_value_type<X<Args...>, N>
{
    typedef decltype(std::get<N>(std::tuple<Args...>())) value_type;
};

/// Overload for Eigen Matrix
template<template<typename Type, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> class X, typename Type,   int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct extract_value_type<X<Type, _Rows,_Cols, _Options, _MaxRows, _MaxCols> >
{
    typedef Type value_type;
    #if ERL_CONSTEXPR_ACTIVE==ERL_TRUE
    static ERL_USE_CONSTEXPR int Rows    = _Rows   ;
    static ERL_USE_CONSTEXPR int Cols    = _Cols   ;
    static ERL_USE_CONSTEXPR int Options = _Options;
    static ERL_USE_CONSTEXPR int MaxRows = _MaxRows;
    static ERL_USE_CONSTEXPR int MaxCols = _MaxCols;
    #endif
};


/// Overload for Rotmat and Transform types

template<template<typename Type, int MajorType> class X, typename Type, int MajorType>
struct extract_value_type<X<Type, MajorType>>
{
    typedef Type value_type;
};

template<template<typename T1, typename T2, unsigned M> class X, typename T1, typename T2, unsigned M, int N>
struct extract_value_type<X<T1,T2, M>,N>
{
    typedef decltype(std::get<N>(std::tuple<T1,T2>())) value_type;
};

/// Convert enum class to int
template <typename Enumeration>
ERL_USE_CONSTEXPR inline auto as_int(const Enumeration value) -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

}

#endif
