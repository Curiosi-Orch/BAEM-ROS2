/*
 * Copyright (c) 2013-2017 Gauthier Gras <gauthier.gras@gmail.com>
 * Copyright (c) 2013-2017 Konrad Leibrandt <konrad.lei@gmx.de>
 * Licensed under the MIT license. See the license file LICENSE.
*/

#ifndef ERL_DUALQUATERNION_H
#define ERL_DUALQUATERNION_H

#include "Erl/_forwards.h"

namespace Erl
{

typedef DualQuaternion<double> DualQuaterniond;
typedef DualQuaternion<float>  DualQuaternionf;

template<class T> struct ScrewParameters
{
    T theta;      /// Angle of rotation
    T dist;       /// Translation along the axis
    Vector3<T> l; /// Direction vector
    Vector3<T> m; /// Moment vector
};

template<class T> struct DualNumber
{
    T real;
    T dual;

    inline DualNumber ()  {}
    inline DualNumber (const DualNumber &Dn)
        : real(Dn.real)
        , dual(Dn.dual)
    {}
    inline DualNumber& operator = (const DualNumber &Dn)
    {
        real = Dn.real;
        dual = Dn.dual;
        return *this;
    }
    inline DualNumber (const T &Real, const T &Dual)
        : real(Real)
        , dual(Dual)
    {}
};
namespace details
{
    template <class T> class dualquaternion_determine_alignment { protected: Quaternion<T> qr; Quaternion<T> qd; };
}
template<class T> class
ERL_ALIGNAS_DUALQUATERNION
DualQuaternion
{

protected:

    Quaternion<T> qr;
    Quaternion<T> qd;

public:

// General constructors ===================================================================================

    inline DualQuaternion () {}

    inline DualQuaternion (const DualQuaternion &Dq)
        : qr(Dq.qr)
        , qd(Dq.qd)
    {}

    inline DualQuaternion& operator = (const DualQuaternion &Dq)
    {
        qr = Dq.qr;
        qd = Dq.qd;
        return *this;
    }

    inline DualQuaternion (const T &wr, const T &xr, const T &yr, const T &zr, const T &wd, const T &xd, const T &yd, const T &zd)
        : qr(wr, xr, yr, zr)
        , qd(wd, xd, yd, zd)
    {}

    inline DualQuaternion (const Quaternion<T> &Qr, const Quaternion<T> &Qd)
        : qr(Qr)
        , qd(Qd)
    {}

    template<typename OtherType> inline DualQuaternion (const Quaternion<OtherType> &Qr, const Quaternion<OtherType> &Qd)
        : qr(Qr.template cast<T>())
        , qd(Qd.template cast<T>())
    {}

    inline DualQuaternion (const Transform<T> &Tr)
        : qr(Tr.getQuaternion())
        , qd(((T)0.5)*Quaternion<T>(Tr.getTranslation())*qr)
    {}

    template<typename OtherType> inline DualQuaternion (const Transform<OtherType> &Tr)
        : qr(Tr.getQuaternion().template cast<T>())
        , qd(0.5*Quaternion<T>(Tr.getTranslation())*qr)
    {}

    inline DualQuaternion& operator = (const Transform<T> &Tr)
    {
        qr = Tr.getQuaternion().template cast<T>();
        qd = 0.5*Quaternion<T>(Tr.getTranslation())*qr;
    }

    template<typename OtherType> inline DualQuaternion& operator = (const Transform<OtherType> &Tr)
    {
        qr = Tr.getQuaternion();
        qd = 0.5*Quaternion<T>(Tr.getTranslation())*qr;
    }

    template<typename OtherType> inline DualQuaternion (const ScrewParameters<OtherType> &param)
    {
        T st2 = std::sin(param.theta/2);
        T ct2 = std::cos(param.theta/2);
        T pd2 = param.dist/2;
        qr = Quaternion<T>(ct2, param.l(0)*st2, param.l(1)*st2, param.l(2)*st2);
        qd = Quaternion<T>(-pd2*st2, param.m(0)*st2 + pd2*ct2*param.l(0), param.m(1)*st2 + pd2*ct2*param.l(1), param.m(2)*st2 + pd2*ct2*param.l(2));
    }

    template<typename OtherType> inline DualQuaternion& operator = (const ScrewParameters<OtherType> &param)
    {
        T st2 = std::sin(param.theta/2);
        T ct2 = std::cos(param.theta/2);
        T pd2 = param.dist/2;
        qr = Quaternion<T>(ct2, param.l(0)*st2, param.l(1)*st2, param.l(2)*st2);
        qd = Quaternion<T>(-pd2*st2, param.m(0)*st2 + pd2*ct2*param.l(0), param.m(1)*st2 + pd2*ct2*param.l(1), param.m(2)*st2 + pd2*ct2*param.l(2));
        return *this;
    }

// From Rotation Type ========================================================================================

    template<typename OtherType> inline DualQuaternion (const Eigen::Matrix<OtherType,3,3>& Rotation)
        : qr(Quaternion<T>(Rotation))
        , qd(0,0,0,0)
    {}

    template<typename OtherType> inline DualQuaternion (const Quaternion<OtherType> &quaternion)
        : qr(quaternion)
        , qd(0,0,0,0)
    {}

// From translation ======================================================================================

    template<typename OtherType> inline DualQuaternion (const Eigen::Matrix<OtherType,3,1>& Translation)
        : qr(1,0,0,0)
        , qd(0.5*Translation)
    {}

// From pointer ==========================================================================================

    template<typename OtherType> static inline DualQuaternion fromPointer(const OtherType *dataCombined)
    {
        return DualQuaternion( Quaternion<OtherType>::fromPointer(dataCombined),
                               Quaternion<OtherType>::fromPointer(dataCombined+4));
    }

    template<typename OtherType> static inline DualQuaternion fromPointer(const OtherType *dataQ1, const OtherType *dataQ2)
    {
        return DualQuaternion( Quaternion<OtherType>::fromPointer(dataQ1),
                               Quaternion<OtherType>::fromPointer(dataQ2));
    }

// Operators ===========================================================================================

    inline DualQuaternion operator + (const DualQuaternion &Dq) const
    {
        return DualQuaternion(qr + Dq.qr, qd + Dq.qd);
    }

    inline DualQuaternion operator - (const DualQuaternion &Dq) const
    {
        return DualQuaternion(qr - Dq.qr, qd - Dq.qd);
    }

    inline DualQuaternion& operator += (const DualQuaternion &Dq)
    {
        qr += Dq.qr;
        qd += Dq.qd;
        return *this;
    }

    inline DualQuaternion& operator -= (const DualQuaternion &Dq)
    {
        qr -= Dq.qr;
        qd -= Dq.qd;
        return *this;
    }

    inline DualQuaternion operator + () const
    {
        return *this;
    }

    inline DualQuaternion operator - () const
    {
        return DualQuaternion(-qr, -qd);
    }

    inline DualQuaternion operator * (const DualQuaternion &Dq) const
    {
        return DualQuaternion(qr*Dq.qr, qr*Dq.qd + qd*Dq.qr);
    }

    inline DualQuaternion& operator *= (const DualQuaternion &Dq)
    {
        qr = qr*Dq.qr;
        qd = qr*Dq.qd + qd*Dq.qr;
        return *this;
    }

    template<typename OtherType> inline DualQuaternion operator * (const OtherType &Scalar) const
    {
        return DualQuaternion(Scalar*qr, Scalar*qd);
    }

    template<typename OtherType> inline DualQuaternion operator *= (const OtherType &Scalar)
    {
        qr *= Scalar;
        qd *= Scalar;
        return *this;
    }

    template<typename OtherType> inline DualQuaternion operator / (const OtherType &Scalar) const
    {
        ERL_ASSERT(Scalar != 0 && "Dividing dual quaternion by zero");
        return DualQuaternion(qr/Scalar, qd/Scalar);
    }

    template<typename OtherType> inline DualQuaternion operator /= (const OtherType &Scalar)
    {
        ERL_ASSERT(Scalar != 0 && "Dividing dual quaternion by zero");
        qr /= Scalar;
        qd /= Scalar;
        return *this;
    }

    template<typename OtherType> inline friend DualQuaternion operator * (const OtherType &Scalar, const DualQuaternion &Dq)
    {
        return Dq*Scalar;
    }

    inline bool operator == (const DualQuaternion &Dq) const
    {
        return (qr == Dq.qr && qd == Dq.qd);
    }

    inline bool operator != (const DualQuaternion &Dq) const
    {
        return (qr != Dq.qr || qd != Dq.qd);
    }

    inline friend std::ostream& operator << (std::ostream &out, const DualQuaternion &Dq)
    {
        out << Dq.qr << " " << Dq.qd;
        return out;
    }

// Accessors ===========================================================================================

    inline Quaternion<T> Qr () const
    {
        return qr;
    }

    inline Quaternion<T> Qd () const
    {
        return qd;
    }

    inline void Qr (const Quaternion<T> &Qin)
    {
        qr = Qin;
    }

    inline void Qd (const Quaternion<T> &Qin)
    {
        qd = Qin;
    }

// Getters =============================================================================================

    template<int MajorType> inline Transform<T, MajorType> getTransform () const
    {
        return Transform<T, MajorType>(qr, T(2)*(qd*qr.conj()).getVector());
    }

    inline Vector3<T> getVector () const
    {
        return qd.getVector();
    }

    inline ScrewParameters<T> getScrewParameters() const
    {
        T wr = qr.w();
        T wd = qd.w();
        Vector3<T> vr = qr.getVector();
        Vector3<T> vd = qd.getVector();
        T vrn = vr.norm();
#ifdef _DEBUG
        assert(vrn != 0 && "real vector norm = 0 for screw parameters");
#endif

        ScrewParameters<T> param;
        param.theta = 2*std::acos(qr.wr);
        param.dist = -2*wd/vrn;
        param.l = vr/vrn;
        param.m = (vd - param.l*param.dist*wr/2)/vrn;

        return param;
    }

// Functions ============================================================================================

    /// The functions below use the name "norm" but refer to the magnitude.

    inline DualNumber<T> norm () const
    {
        T realNorm = qr.norm();
        ERL_ASSERT(realNorm != 0 && "Dual quaternion real part norm = 0");
        return DualNumber<T>(realNorm, qr.dot(qd)/realNorm);
    }

    inline DualQuaternion normalized () const
    {
        T realNorm = qr.norm();
        ERL_ASSERT(realNorm != 0 && "Dual quaternion real part norm = 0");
        T invNorm = 1.0/realNorm;
        Quaternion<T> qar, qad;
        qar = qr*invNorm;
        qad = qd*invNorm;
        qad -= qar*(qar.dot(qad));

        return DualQuaternion(qar, qad);
    }

    inline void normalize ()
    {
        T realNorm = qr.norm();
        ERL_ASSERT(realNorm != 0 && "Dual quaternion real part norm = 0");
        T invNorm = 1.0/realNorm;
        qr *= invNorm;
        qd *= invNorm;
        qd -= qr*(qr.dot(qd));
    }

    /// Inverse not allowed, conjugate should be used with unit quaternions instead.

    inline DualQuaternion conj () const
    {
        return DualQuaternion(qr.conj(), qd.conj());
    }

    template <typename OtherType> inline Vector3<T> transformVector (const Eigen::Matrix<OtherType,3,1>& vector) const
    {
        ERL_ASSERT(qr.norm() != 1 && "Dual quaternion not a unit dual quaternion");
        DualQuaternion result = (*this)*DualQuaternion(vector)*conj();
        return result.getVector();
    }

    template <typename OtherType> inline DualQuaternion<OtherType> cast () const
    {
        return DualQuaternion<OtherType> (qr, qd);
    }

    inline static DualQuaternion Identity ()
    {
        return DualQuaternion(1,0,0,0,0,0,0,0);
    }

    inline static DualQuaternion Zero ()
    {
        return DualQuaternion(0,0,0,0,0,0,0,0);
    }

};

}

#endif
