/*
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_ANGLE_H
#define ERL_ANGLE_H

#include <complex>
#include "Erl/Utility/util.h"
#include "Erl/Core/vector3.h"

namespace Erl
{
template<class T> class Angle;

typedef Angle<double> Angled;
typedef Angle<float > Anglef;

template<class T>  class
#ifdef ERL_ANGLE_ALIGNMENT
        alignas(ERL_ANGLE_ALIGNMENT)
#endif
Angle
{
public:
    typedef std::complex<T> cmplex;
// Essential ======================================================================================================
    inline Angle() {}
// Constructors ====================================================================================================
    inline Angle(T angle_rad) : m_arg(wrapToPi(angle_rad)) {}

    inline Angle(T real, T imag) : m_arg(std::arg<T>(cmplex(real,imag)))    {}

    inline Angle(const cmplex& angle_cmplx) : m_arg(std::arg<T>(angle_cmplx))    {}

    inline Angle(const Angle& angle) : m_arg(angle.m_arg)    {}

    template <typename OtherType>
    inline Angle (const OtherType *dataIn) : m_arg(wrapToPi(T(dataIn[0]))) {}
// Operators ========================================================================================================

   inline Angle operator + ( const cmplex& angle_cmplx ) const
   {
       return Angle(wrapToPi(m_arg+std::arg<T>(angle_cmplx)));
   }

   inline Angle operator - ( const cmplex& angle_cmplx ) const
   {
       return Angle(wrapToPi(m_arg-std::arg<T>(angle_cmplx)));
   }

   inline Angle operator + ( const Angle& angle) const
   {
       return Angle(wrapToPi(m_arg+angle.m_arg));
   }

   inline Angle operator - ( const Angle& angle ) const
   {
       return Angle(wrapToPi(m_arg-angle.m_arg));
   }

   inline Angle operator + ( const T &fScalar ) const
   {
       return Angle(wrapToPi(m_arg+fScalar));
   }

   inline Angle operator - ( const T &fScalar ) const
   {
       return Angle(wrapToPi(m_arg-fScalar));
   }

   inline Angle operator * ( const T &fScalar ) const
   {
       return Angle(wrapToPi(m_arg*fScalar));
   }

   inline Angle operator / ( const T &fScalar ) const
   {
       ERL_ASSERT( fScalar != 0.0 && "Angle divided by zero");
       return Angle(wrapToPi(m_arg/fScalar));
   }

   inline friend Angle operator * (const T &fScalar, const Angle& angle_rad )
   {
       return Angle(angle_rad*fScalar);
   }

   inline friend Angle operator / (const T &fScalar, const Angle& angle_rad )
   {
       return Angle(angle_rad/fScalar);
   }

   inline Angle& operator += ( const cmplex& angle_cmplx )
   {
       m_arg = wrapToPi(m_arg+std::arg<T>(angle_cmplx));
       return *this;
   }

   inline Angle& operator -= ( const cmplex& angle_cmplx )
   {
       m_arg = wrapToPi(m_arg-std::arg<T>(angle_cmplx));
       return *this;
   }

   inline Angle& operator *= ( const T &fScalar )
   {
       m_arg = wrapToPi(m_arg*fScalar);
       return *this;
   }


   inline Angle& operator /= ( const T &fScalar )
   {
       ERL_ASSERT( fScalar != 0.0 && "Vector3 divided by zero");
       m_arg = wrapToPi(m_arg/fScalar);
       return *this;
   }

   inline Angle& operator += ( const T &fScalar )
   {
       m_arg = wrapToPi(m_arg+fScalar);
       return *this;
   }

   inline Angle& operator -= ( const T &fScalar )
   {
       m_arg = wrapToPi(m_arg-fScalar);
       return *this;
   }

   inline Angle& operator += ( const Angle &angle )
   {
       m_arg = wrapToPi(m_arg+angle.m_arg);
       return *this;
   }

   inline Angle& operator -= ( const Angle &angle )
   {
       m_arg = wrapToPi(m_arg-angle.m_arg);
       return *this;
   }

   inline bool operator == ( const cmplex& angle_cmplx ) const
   {
       return m_arg == std::arg<cmplex>(angle_cmplx);
   }

   inline bool operator != ( const cmplex& angle_cmplx ) const
   {
       return m_arg != std::arg<cmplex>(angle_cmplx);
   }

   inline bool operator == ( const T& _arg ) const
   {
       return m_arg == _arg;
   }

   inline bool operator != ( const T& _arg ) const
   {
       return m_arg != _arg;
   }

   inline bool operator == ( const Angle& _arg ) const
   {
       return m_arg == _arg.m_arg;
   }

   inline bool operator != ( const Angle& _arg ) const
   {
       return m_arg != _arg.m_arg;
   }

   inline T operator () () const
   {
       return m_arg;
   }

   friend std::ostream & operator<< (std::ostream& os, const Angle &obj)
   {
       os << obj.m_arg;
       return os;
   }

   // Accessors ======================================================================================================

   inline T distanceTo (const cmplex &angle_cmplx) const
   {
       return std::abs<T>(Erl::wrapToPi<T>(std::abs<T>(m_arg - std::arg<T>(angle_cmplx))));
   }

   inline T distanceTo (const T & fScalar) const
   {
       return std::abs<T>(wrapToPi(std::abs<T>(m_arg- fScalar)));
   }

   inline T signedDistanceTo (const T & fScalar) const
   {
       return signedDistance(m_arg,fScalar);
   }

   inline T distanceTo (const Angle & angle) const
   {
       return std::abs<T>(wrapToPi(std::abs<T>(m_arg- angle.m_arg)));
   }

   inline cmplex complex() const
   {
       return cmplex(std::cos(m_arg),std::sin(m_arg));
   }

   inline T real() const
   {
       std::cos(m_arg);
   }

   inline T imag() const
   {
       std::sin(m_arg);
   }
   inline T arg() const
   {
       return m_arg;
   }

   template<typename _IIter>
   inline static Angle mean(_IIter first, _IIter last)
   {
       cmplex _AngleRet(T(0),T(0));
       size_t _n(0);
       while(first != last)
       {
           _AngleRet += cmplex( std::cos(first->arg),
                                std::sin(first->arg)
                              );
           first++; _n++;
       }
       _AngleRet/= _n;
       return Angle(_AngleRet);
   }

    inline static Angle mean(const Angle& _A0,const Angle& _A1)
    {
        cmplex c_AngleRet(std::cos(_A0.m_arg),std::sin(_A0.m_arg));
                  c_AngleRet+=cmplex(std::cos(_A1.m_arg),std::sin(_A1.m_arg));
                  c_AngleRet/= 2.;
           return Angle(c_AngleRet);
    }

    inline static T mean(const T& _A0,const T& _A1)
    {
        return std::atan2((std::sin(_A0)+std::sin(_A1))/T(2),(std::cos(_A0)+std::cos(_A1))/T(2));
    }

    inline static T distance(const T& _A0,const T& _A1)
    {
        return std::abs<T>(wrapToPi(_A0-_A1));
    }

    inline static T signedDistance(const T& _A0,const T& _A1)
    {
        T c_ADiff = wrapToPi(_A0) - wrapToPi(_A1);
        c_ADiff +=  (c_ADiff>T(ERL_PI)) ? T(-2.*ERL_PI) : (c_ADiff<T(-ERL_PI)) ? T(2.*ERL_PI) : T(0);
        return c_ADiff;
    }

    template<typename _IIterMin,typename _IIterSub,typename _IIterDiff>
    inline static void signedDistance(_IIterMin _firstMinuend, _IIterMin _lastMinuend, _IIterSub _firstSubtrahend, _IIterDiff _firstDifferenceOut )
    {
        while(_firstMinuend!=_lastMinuend)
            *_firstDifferenceOut++ = signedDistance(*_firstMinuend++,*_firstSubtrahend++);
    }


    inline static  T wrapToPi(T _angle_rad)
    {
        _angle_rad+=T(ERL_PI);
        return (std::fmod(_angle_rad, T(2.*ERL_PI)) + ( (_angle_rad < 0) ? ERL_PI : -ERL_PI ) );
        //return (std::fmod(c_angle_rad, T(2.*ERL_PI)) - Erl::signum<T>(c_angle_rad)*(ERL_PI)) ;
    }

    template<typename _IIter>
    inline static void wrapToPi(_IIter _firstAngle, _IIter _lastAngle)
    {
        while(_firstAngle!=_lastAngle)
        {
            *_firstAngle = wrapToPi(*_firstAngle);
            _firstAngle++;
        }
    }

    inline static Angle cuttingAngleSign(const Erl::Vector3<T>& _vecPri, const Erl::Vector3<T>& _vecSec)
    {
        Erl::Vector3<T> c_VecPri(_vecPri.normalized());
        Erl::Vector3<T> c_VecSec(_vecSec.normalized());
        Erl::Vector3<T> c_Axis   (c_VecPri.cross(c_VecSec));
        Erl::Vector3<T> c_AxisAbs(std::fabs(c_Axis[0]), std::fabs(c_Axis[1]),std::fabs(c_Axis[2]));
        T c_Sign = Erl::signum(c_Axis[(c_AxisAbs[0]>c_AxisAbs[1]) ? ( c_AxisAbs[0]>c_AxisAbs[2] ? 0 : 2 ) : (c_AxisAbs[1]>c_AxisAbs[2] ? 1 : 2)]) ;
        return c_Sign*std::atan2(c_VecPri.cross(c_VecSec).norm(),c_VecPri.dot(c_VecSec));
    }
    inline static Angle cuttingAngle(const Erl::Vector3<T>& _vecPri, const Erl::Vector3<T>& _vecSec)
    {

        Erl::Vector3<T> c_VecPri(_vecPri.normalized());
        Erl::Vector3<T> c_VecSec(_vecSec.normalized());
        return std::atan2(c_VecPri.cross(c_VecSec).norm(),c_VecPri.dot(c_VecSec));
    }
    inline static Angle cuttingAngle(const Erl::Vector3<T>& _vecPri, const Erl::Vector3<T>& _vecSec, Erl::Vector3<T>& _axisReturn)
    {
        Erl::Vector3<T> c_VecPri(_vecPri.normalized());
        Erl::Vector3<T> c_VecSec(_vecSec.normalized());
        _axisReturn = c_VecPri.cross(c_VecSec);
        T _norm = _axisReturn.norm();
        if(_norm > Erl::Constants<T>::Zero_Tolerance)
            _axisReturn = _axisReturn/_norm;

        return std::atan2(_norm,c_VecPri.dot(c_VecSec));
    }

private:
    T m_arg;

};


}
#endif // ERL_ANGLE_H
